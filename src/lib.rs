//! # ab3dmot 模型

mod kalman_filter;

use kalman_filter::TrackerKF;
use mot_data::{BBox3D, ObjectType, input::{Frame as IFrame}, output::{Object as OObject, Frame as OFrame}};

use geo::{intersects::Intersects, polygon, prelude::{Area}};
use geo_clipper::{Clipper};
use itertools::Itertools;
use pathfinding::{kuhn_munkres::{kuhn_munkres}, matrix::{Matrix}, prelude::Weights};

use uuid::{Uuid};
use names::{Generator};

/* 常数定义区开始 */

/// 匈牙利算法的放大系数
const MAGNIFY: f32 = 1e6;

/* 常数定义区结束 */

/* 私有函数定义区开始 */

/// 用放大方法将对 float 型的最优匹配问题转为对 int 型的最优匹配问题
fn kuhn_munkres_4_f32(weight_matrix_f64: &Matrix<f32>) -> (i64, Vec<usize>) {
    let mut weight_matrix_i64 = Matrix::new(weight_matrix_f64.rows(), weight_matrix_f64.columns(), 0i64);
    for (i, j) in (0..weight_matrix_f64.rows()).cartesian_product(0..weight_matrix_f64.columns()) {
        weight_matrix_i64[&(i, j)] = (weight_matrix_f64[&(i, j)]*MAGNIFY) as i64;
    }
    kuhn_munkres(&weight_matrix_i64)
}

/// ## 匈牙利算法最优匹配
/// 返回值中：
/// 1. `Vec<(usize, usize, f32)>` 对应 trk_idx, det_idx, iou3d
/// 2. `Vec<usize>` 对应 unmatched_trks 的 idx
/// 3. `Vec<usize>` 对应 unmatched_dets 的 idx
fn associate_detections_to_trackers(
    trks: &Vec<BBox3D::CornerPoints>,
    dets: &Vec<BBox3D::CornerPoints>,
    iou_threshold: f32,
) -> (Vec<(usize, usize, f32)>, Vec<usize>, Vec<usize>) /* matched, unmatched_trks, unmatched_dets*/ {
    // matched: (trk_idx, det_idx, iou3d)
    if dets.is_empty() {
        // dets 为空则
        (Vec::<(usize, usize, f32)>::new(), (0..dets.len()).collect(), Vec::<usize>::new())
    } else {
        // pathfinding crate 中的匈牙利算法要求行数小于等于列数，即每个行都有列与之对应，每个行对应的列不相同，而列可以没有行与之对应
        // 行为任务，列为员工，所有任务必须完成，而不需要所有员工都有任务做
        if trks.len() <= dets.len() {
            // 当 trks 数量少于 dets，则将 trk_id 置为匈牙利算法的行

            // 这里的 Matrix 是 pathfinding 中的 Matrix
            let mut iou_matrix = Matrix::new(trks.len(), dets.len(), 0f32);
            for ((trk_i, each_trk), (det_j, each_det), ) in
            trks.iter().enumerate().cartesian_product(dets.iter().enumerate()) {
                // 为每个 trk 和 每个 det 的配对都计算 iou3d
                iou_matrix[&(trk_i, det_j)] = iou3d(each_det, each_trk).0;
            }

            // 得到以原 trks 向量中每一个 trk_id 作为这个 Vec 中的 idx，赋于其对应的 det 在 dets Vec 中的 idx
            let matched__idx_as_trk_idx__value_as_det_idx = kuhn_munkres_4_f32(&iou_matrix).1;

            let (mut unmatched_trks, mut unmatched_dets, ) = (Vec::new(), Vec::new(), );
            for det_idx in 0..dets.len() {
                if !matched__idx_as_trk_idx__value_as_det_idx.contains(&det_idx) {
                    unmatched_dets.push(det_idx);
                }
            }
            let mut matched = Vec::new();

            // 进一步地筛选 iou3d 过小的匹配
            for (trk_idx, det_idx) in matched__idx_as_trk_idx__value_as_det_idx.into_iter().enumerate() {
                if iou_matrix.at(trk_idx, det_idx) < iou_threshold {
                    unmatched_trks.push(trk_idx);
                    unmatched_dets.push(det_idx);
                } else {
                    // 所以最后返回的 matched Vec<(usize, usize, f32)> 来说两个维度都有可能是稀疏的
                    matched.push((trk_idx, det_idx, f32::from(iou_matrix[&(trk_idx, det_idx)])));
                }
            } 
            (matched, unmatched_trks, unmatched_dets)
        } else {
            // 以 det 作为行，trk 作为列

            let mut iou_matrix = Matrix::new(dets.len(), trks.len(), 0f32);
            for ((trk_i, each_trk), (det_j, each_det), ) in
            trks.iter().enumerate().cartesian_product(dets.iter().enumerate()) {
                iou_matrix[&(det_j, trk_i)] = iou3d(each_det, each_trk).0;
            }
            let matched__idx_as_det_idx__value_as_trk_idx = kuhn_munkres_4_f32(&iou_matrix).1;
            let (mut unmatched_trks, mut unmatched_dets, ) = (Vec::new(), Vec::new(), );
            for trk_idx in 0..trks.len() {
                if !matched__idx_as_det_idx__value_as_trk_idx.contains(&trk_idx) {
                    unmatched_trks.push(trk_idx);
                }
            }
            let mut matched = Vec::new();
            for (det_idx, trk_idx) in matched__idx_as_det_idx__value_as_trk_idx.into_iter().enumerate() {
                if iou_matrix.at(det_idx, trk_idx) < iou_threshold {
                    unmatched_dets.push(det_idx);
                    unmatched_trks.push(trk_idx);
                } else {
                    matched.push((trk_idx, det_idx, f32::from(iou_matrix[&(det_idx, trk_idx)])));
                }
            } 
            (matched, unmatched_trks, unmatched_dets)
        }
    }
}

fn iou3d(bbox1: &BBox3D::CornerPoints, bbox2: &BBox3D::CornerPoints) -> (
    f32,  // 3d IOU 值
    f32,  // 底面积 2d IOU 值
) {
    let (
        BBox3D::CornerPoints(b10, b11, b12, b13, b14, ..),
        BBox3D::CornerPoints(b20, b21, b22, b23, b24, ..),
    ) = (bbox1, bbox2);
    let (bbox1_base, bbox2_base) = (
        polygon![
            (x: b10[0] as f64, y: b10[2] as f64),
            (x: b11[0] as f64, y: b11[2] as f64),
            (x: b12[0] as f64, y: b12[2] as f64),
            (x: b13[0] as f64, y: b13[2] as f64),
        ], polygon![
            (x: b20[0] as f64, y: b20[2] as f64),
            (x: b21[0] as f64, y: b21[2] as f64),
            (x: b22[0] as f64, y: b22[2] as f64),
            (x: b23[0] as f64, y: b23[2] as f64),
        ],
    );
    // 求出俩底面积
    let (bbox1_base_area, bbox2_base_area) = (bbox1_base.unsigned_area() as f32, bbox2_base.unsigned_area() as f32);
    if bbox1_base.intersects(&bbox2_base) {  // 如果有相交部分
        // 两个长方形相交部分数顶多一个
        let base_intersection_area = bbox1_base.intersection(&bbox2_base, 1e6).iter().nth(0).unwrap().unsigned_area() as f32;
    
        // 相交体积的上部和下部
        let (h_intersection_upper, h_intersection_lower) = (
            b14[1].max(b24[1]),
            b10[1].min(b20[1])
        );

        // 相交部分的高
        let h_intersection_len = (h_intersection_upper - h_intersection_lower).abs();

        let (bbox1_vol, bbox2_vol, intersection_vol) = (
            bbox1_base_area * (b14[1] - b10[1]).abs(),
            bbox2_base_area * (b24[1] - b20[1]).abs(),
            base_intersection_area * h_intersection_len,
        );

        (
            intersection_vol/(bbox1_vol + bbox2_vol - intersection_vol),  // 体积的交/体积的并
            base_intersection_area/(bbox1_base_area + bbox2_base_area - base_intersection_area),  // 底面积的交/底面积的并
        )
    } else {
        (
            0f32,
            0f32,
        )
    }
}

/* 私有函数定义区结束 */

/* trait 定义区开始 */

// pub enum IFrameOrDeltaT<'a> {
//     SomeIFrame(&'a IFrame),
//     SomeDeltaT(usize),
// }

// pub trait IntoIFrameOrDeltaT {
//     fn into_it<'a>(&'a self) -> IFrameOrDeltaT<'a>;
// }
// impl IntoIFrameOrDeltaT for IFrame {
//     fn into_it<'a>(&'a self) -> IFrameOrDeltaT<'a> {
//         IFrameOrDeltaT::SomeIFrame(self)
//     }
// }
// impl IntoIFrameOrDeltaT for usize {
//     fn into_it(&self) -> IFrameOrDeltaT<'static>
//     {
//         IFrameOrDeltaT::SomeDeltaT(*self)
//     }
// }

pub enum IFrameOrDeltaT<'a> {
    SomeIFrame(&'a IFrame),
    SomeDeltaT(usize),
}

pub trait IntoIFrameOrDeltaT<'a> {
    fn into(self) -> IFrameOrDeltaT<'a>;
}
impl<'a> IntoIFrameOrDeltaT<'a> for &'a IFrame {
    fn into(self) -> IFrameOrDeltaT<'a> {
        IFrameOrDeltaT::SomeIFrame(self)
    }
}
impl IntoIFrameOrDeltaT<'static> for usize {
    fn into(self) -> IFrameOrDeltaT<'static>
    {
        IFrameOrDeltaT::SomeDeltaT(self)
    }
}

/* trait 定义区结束 */

/* 私有结构体定义区开始 */

/// # 追踪每一个轨迹
#[derive(Clone)]
struct Tracker {
    /// Track 的唯一 id
    id: Uuid,
    /// 随机生成的名字
    name: String,
    /// 对象类型（车，行人，等）
    object_type: ObjectType,
    /// 内部的 traker kalman filter
    trk_kf: TrackerKF,
    /// 命中次数
    num_hit: usize,
    /// 自上次命中后失配次数
    unmatched_f_since_l: usize,
    /// iou3d 的历史分数，数量等于 AB3DMOT 中的 frame_count
    /// 如果是是纯预测输出则不会变动
    iou3d_history: Vec<f32>,
}

/* 私有结构体定义区结束 */

/* 共有结构体定义区开始 */

/// ## AB3DMOT 模型
pub struct AB3DMOT {
    /// 自从上次匹配成功后看不到一个 traker n 次，当 n > max_age 就丢弃
    max_age: usize,
    /// 一个新物体持续追踪到 n 次，当 n >= min_hit 就认为是新物体
    min_hit: usize,
    /// 初次估计的误差方差
    initial_estimation_noise_var: [f32; 10],
    /// 匹配的 iou 下限
    iou_threshold: f32,
    /// 已经确定是一个物体了
    mature_trackers: Vec<Tracker>,
    /// 还未确定是一个物体
    immature_trackers: Vec<Tracker>,
    /// 历史上给出的有效检测帧的个数
    frame_count: usize,
}

impl AB3DMOT {
    /// 给出 `max_age`, `min_hit` 和 `iou_threshold` 三个参数
    pub fn new(
        max_age: usize,
        min_hit: usize,
        iou_threshold: f32,
        motion_model_process_conv: [f32; 10],
        observation_conv: [f32; 7],
        initial_estimation_noise_var: [f32; 10],
    ) -> Self {
        TrackerKF::set_convs(motion_model_process_conv, observation_conv);
        Self {
            max_age,
            min_hit,
            initial_estimation_noise_var,
            iou_threshold,
            mature_trackers: Vec::<Tracker>::new(),
            immature_trackers: Vec::<Tracker>::new(),
            frame_count: 0,
        }
    }

    /// 给出一个 `delta_t` 的间隔时间来对某个由 tracker 组成的 vector 中的各个 traker 预测下一状态（相距这一状态 `delta_t` 的时间）
    ///
    /// 以 vector 的形式返回
    #[inline]
    fn get_predictions(trackers: &Vec<Tracker>, delta_t: f32) -> Vec<BBox3D::XYZWHLRotY> {
        trackers.iter().map(
            |x| {
                // 这里需要对 x 中的 trk_kf 指定一个 predict 方法的具体实现
                // 因其同时在 TrackerKF 结构体自身和它所 impl 的 trait 中定义了 predict 方法
                // 下面这里使用的是自己手动在 TrackerKF 自身实现的 predict 方法
                let rst_ovecter = (&x.trk_kf as &TrackerKF).predict(delta_t).state().clone();
                BBox3D::XYZWHLRotY(
                    rst_ovecter[0],
                    rst_ovecter[1],
                    rst_ovecter[2],
                    rst_ovecter[3],
                    rst_ovecter[4],
                    rst_ovecter[5],
                    rst_ovecter[6],
                )
            }
        ).collect()
    }

    /// 通过给出一个 iframe: &mot_data::input::Frame 或是 delta_t: f32 来实现更新
    /// 这里实现了对这两个不同类型引用的重载
    pub fn update<'a>(&mut self, iframe_or_delta_t: impl IntoIFrameOrDeltaT<'a>) -> OFrame::Verbose /*有返回值，但不清楚类型*/ 
    {
        let iframe_or_delta_t = iframe_or_delta_t.into();

        match iframe_or_delta_t {  // 如果给了有效数据
            IFrameOrDeltaT::SomeIFrame(frame) => {
                // 包含一个个三元组的数组
                let data_3d = frame.get_3d_infos();
                let delta_t = frame.delta_t as f32;
                // 先取出三元组的第一个量
                let (objects_type, remain_two): (Vec<_>, Vec<_>) = data_3d.into_iter().map(
                    |x| (x.0, (x.1, x.2))
                ).unzip();
                // 再取出三元组的第二个和第三个量
                let (dets_xyz, _scores): (Vec<_>, Vec<_>) = remain_two.into_iter().unzip();
                // 将所有 xyzwhlroty 转成 cornerpoints
                let dets_corner: Vec<_> = dets_xyz.iter().map(|x| x.to_CornerPoints()).collect();
                // 总 frame 数加一
                self.frame_count += 1;
                
                let mature_tracker_predictions = Self::get_predictions(&self.mature_trackers, delta_t);
                let immature_tracker_predictions = Self::get_predictions(&self.immature_trackers, delta_t);

                // 用这次的检测数据同 mature_trks 进行匹配
                let (
                    matched_with_mature_trks,
                    mature_trks_idx_unmatched,
                    dets_idx_unmatched_with_mature_trks
                ) = associate_detections_to_trackers(
                    &(mature_tracker_predictions.iter().map(|x| x.to_CornerPoints()).collect()),  // 用每个 tracker 的预测值和新出现的 bbox 之间进行匹配
                    &dets_corner,
                    self.iou_threshold
                );
                
                // 对于 mature_trks 中匹配上的 trks
                for (trker_id, det_id, iou) in matched_with_mature_trks {
                    let trker_mut_ref = &mut self.mature_trackers[trker_id];
                    trker_mut_ref.unmatched_f_since_l = 0;
                    trker_mut_ref.trk_kf.update(dets_xyz[det_id], delta_t);  // kalman 状态更新
                    trker_mut_ref.iou3d_history.push(iou);  // iou 历史值更新
                    trker_mut_ref.num_hit += 1;  // 命中数 +1
                }

                // 对于 mature_trks 中未匹配上的 trks
                mature_trks_idx_unmatched.iter().filter_map(|&trker_id | {
                    // 对于每一个没匹配上的 trk 来说
                    let trker_mut_ref = &mut self.mature_trackers[trker_id];
                    trker_mut_ref.unmatched_f_since_l += 1;
                    if trker_mut_ref.unmatched_f_since_l > self.max_age {
                        Some(trker_id)  // 删除超出最大保存时间的 tracker
                    } else {
                        trker_mut_ref.trk_kf.update(mature_tracker_predictions[trker_id], delta_t);  // 拿自己的预测值来更新自己
                        trker_mut_ref.iou3d_history.push(0f32);  // 没有命中
                        None
                    }
                }).sorted_by(|x, y| y.cmp(x))/*这里必须要彻底拿到*/.for_each(|trker_id_needed_2b_removed| {
                    self.mature_trackers.swap_remove(trker_id_needed_2b_removed);
                });
                
                // 用未匹配成功的检测数据同 immature_trks 中的数据进行匹配
                let (
                    matched_with_immature_trks,
                    immature_trks_idx_unmatched,
                    dets_idx_unmatched_with_immature_trks
                ) = associate_detections_to_trackers(
                    &(immature_tracker_predictions.iter().map(|x| x.to_CornerPoints()).collect()),  // 用每个 tracker 的预测值和新出现的 bbox 之间进行匹配
                    &(dets_idx_unmatched_with_mature_trks.iter().map(|x| dets_corner[*x]).collect()),
                    self.iou_threshold
                );
                
                // 对于 immature_trks 中成功同剩余检测数据匹配上的 trks 来说
                matched_with_immature_trks.iter().filter_map(|&(trker_id, det_id, iou)| {
                    let trker_mut_ref = &mut self.immature_trackers[trker_id];
                    trker_mut_ref.unmatched_f_since_l = 0;
                    trker_mut_ref.trk_kf.update(dets_idx_unmatched_with_mature_trks.iter().map(|&x| dets_xyz[x]).nth(det_id).unwrap(), delta_t);  // kalman 状态更新
                    trker_mut_ref.iou3d_history.push(iou);  // iou 历史值更新
                    trker_mut_ref.num_hit += 1;  // 命中数 +1
                    if trker_mut_ref.num_hit >= self.min_hit {  // 如果可以「孵化」了
                        // 将 immature_trks 中的一条 trk 移送 到 mature_trks 中
                        self.mature_trackers.push(self.immature_trackers[trker_id].clone());
                        Some(trker_id)
                    } else {
                        None
                    }
                }).collect::<Vec<usize>>().into_iter().chain(immature_trks_idx_unmatched.iter().filter_map(|&trker_id| {
                    //对于 immature_trks 中未能成功匹配上的 trks 来说
                    let trker_mut_ref = &mut self.immature_trackers[trker_id];
                    trker_mut_ref.unmatched_f_since_l += 1;
                    if trker_mut_ref.unmatched_f_since_l > self.max_age {
                        Some(trker_id)  // 删除超出最大保存时间的 tracker
                    } else {
                        trker_mut_ref.trk_kf.update(immature_tracker_predictions[trker_id], delta_t);  // 拿自己的预测值来更新自己
                        trker_mut_ref.iou3d_history.push(0f32);  // 没有命中
                        None
                    }
                })).sorted_by(|x, y| y.cmp(x)).for_each(|trker_id_needed_2b_removed| {
                    self.immature_trackers.swap_remove(trker_id_needed_2b_removed);
                });

                let mut name_generator = Generator::default();  // 随机名称生成器

                // 对于最后都没有成功匹配上任何一种 trks 来说，认为可能是新物体
                for det_id in dets_idx_unmatched_with_immature_trks {
                    self.immature_trackers.push(
                        Tracker {
                            id: Uuid::new_v4(),
                            name: name_generator.next().unwrap(),
                            object_type: objects_type[det_id],
                            trk_kf: TrackerKF::new(
                                dets_xyz[det_id],
                                self.initial_estimation_noise_var,
                            ),
                            num_hit: 0,  // 第一次出现不认为是命中
                            unmatched_f_since_l: 0,  // 第一次出现也不认为是失配
                            iou3d_history: Vec::<f32>::new(),
                        }
                    )
                }  // 新的 tracker


                OFrame::Verbose {
                    objects: self.mature_trackers.iter().map(  // 只将命中次数大于命中次数下限的
                        |x| {
                            OObject::Verbose {
                                id: x.id,
                                name: x.name.clone(),
                                object_type: x.object_type,
                                bbox_2d: None,
                                bbox_3d: Some(x.trk_kf.get_latest_observation()),
                                num_hit: x.num_hit,
                                unmatched_f_since_l: x.unmatched_f_since_l,
                                iou3d_history: x.iou3d_history.clone(),
                                score: *x.iou3d_history.last().unwrap(),
                            }
                        }
                    ).collect()
                }
            },
            IFrameOrDeltaT::SomeDeltaT(delta_t) => {  // 传入空的 frame，意味着要预测一帧
                let delta_t = delta_t as f32;
                let mature_tracker_predictions = Self::get_predictions(&self.mature_trackers, delta_t);
                let immature_tracker_predictions = Self::get_predictions(&self.immature_trackers, delta_t);
                for (each_trk, each_prediction) in
                    self.mature_trackers.iter_mut().chain(self.immature_trackers.iter_mut()).zip(
                        mature_tracker_predictions.iter().chain(immature_tracker_predictions.iter())
                ) {
                    each_trk.trk_kf.update(*each_prediction, delta_t);
                    // 暂不更新 iou3d 的值
                    // each_trk.iou3d_history = 0f32;
                }
                OFrame::Verbose {
                    objects: self.mature_trackers.iter().map(  // 只将命中次数大于命中次数下限的
                        |x| {
                            OObject::Verbose {
                                id: x.id,
                                name: x.name.clone(),
                                object_type: x.object_type,
                                bbox_2d: None,
                                bbox_3d: Some(x.trk_kf.get_latest_observation()),
                                num_hit: x.num_hit,
                                unmatched_f_since_l: x.unmatched_f_since_l,
                                iou3d_history: x.iou3d_history.clone(),
                                score: *x.iou3d_history.last().unwrap(),
                            }
                        }
                    ).collect()
                }
            },
        }
    }
}

/* 共有结构体定义区结束 */