pub mod data;

mod kalman_filter;

use kalman_filter::TrackerKF;
use crate::data::{BBox3D, ObjectType, input::{Frame as IFrame}, output::{Object as OObject, Frame as OFrame}};

use geo::{intersects::Intersects, polygon, prelude::{Area}};
use geo_clipper::{Clipper};
use itertools::Itertools;
use pathfinding::{kuhn_munkres::{kuhn_munkres}, matrix::{Matrix}, prelude::Weights};
use ordered_float::{OrderedFloat};

use uuid::{Uuid};
use names::{Generator};

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
            let mut iou_matrix = Matrix::new(trks.len(), dets.len(), OrderedFloat(0f32));
            for ((trk_i, each_trk), (det_j, each_det), ) in
            trks.iter().enumerate().cartesian_product(dets.iter().enumerate()) {
                // 为每个 trk 和 每个 det 的配对都计算 iou3d
                iou_matrix[&(trk_i, det_j)] = OrderedFloat(iou3d(each_det, each_trk).0);
            }

            // 得到以原 trks 向量中每一个 trk_id 作为这个 Vec 中的 idx，赋于其对应的 det 在 dets Vec 中的 idx
            let matched__idx_as_trk_idx__value_as_det_idx = kuhn_munkres(&iou_matrix).1;

            let (mut unmatched_trks, mut unmatched_dets, ) = (Vec::new(), Vec::new(), );
            for det_idx in 0..dets.len() {
                if !matched__idx_as_trk_idx__value_as_det_idx.contains(&det_idx) {
                    unmatched_dets.push(det_idx);
                }
            }
            let mut matched = Vec::new();

            // 进一步地筛选 iou3d 过小的匹配
            for (trk_idx, det_idx) in matched__idx_as_trk_idx__value_as_det_idx.into_iter().enumerate() {
                if iou_matrix.at(trk_idx, det_idx) < OrderedFloat(iou_threshold) {
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

            let mut iou_matrix = Matrix::new(dets.len(), trks.len(), OrderedFloat(0f32));
            for ((trk_i, each_trk), (det_j, each_det), ) in
            trks.iter().enumerate().cartesian_product(dets.iter().enumerate()) {
                iou_matrix[&(det_j, trk_i)] = OrderedFloat(iou3d(each_det, each_trk).0);
            }
            let matched__idx_as_det_idx__value_as_trk_idx = kuhn_munkres(&iou_matrix).1;
            let (mut unmatched_trks, mut unmatched_dets, ) = (Vec::new(), Vec::new(), );
            for trk_idx in 0..trks.len() {
                if !matched__idx_as_det_idx__value_as_trk_idx.contains(&trk_idx) {
                    unmatched_trks.push(trk_idx);
                }
            }
            let mut matched = Vec::new();
            for (det_idx, trk_idx) in matched__idx_as_det_idx__value_as_trk_idx.into_iter().enumerate() {
                if iou_matrix.at(det_idx, trk_idx) < OrderedFloat(iou_threshold) {
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
    let (bbox1_base_area, bbox2_base_area) = (bbox1_base.unsigned_area() as f32, bbox2_base.unsigned_area() as f32);
    if bbox1_base.intersects(&bbox2_base) {
        let base_intersection_area = bbox1_base.intersection(&bbox2_base, 1e6).iter().nth(0).unwrap().unsigned_area() as f32;
    
        let (h_intersection_upper, h_intersection_lower) = ( b14[1].max(b24[1]), b10[1].min(b20[1]));

        let h_intersection_len = (h_intersection_upper - h_intersection_lower).abs();

        let (bbox1_vol, bbox2_vol, intersection_vol) = (
            bbox1_base_area * (b14[1] - b10[1]).abs(),
            bbox2_base_area * (b24[1] - b20[1]).abs(),
            base_intersection_area * h_intersection_len,
        );

        (
            intersection_vol/(bbox1_vol + bbox2_vol - intersection_vol),
            base_intersection_area/(bbox1_base_area + bbox2_base_area - base_intersection_area),
        )
    } else {
        (
            0f32,
            0f32,
        )
    }
}

struct Tracker {
    id: Uuid,
    name: String,
    object_type: ObjectType,
    bbox_3d: BBox3D::XYZLHWRotY,
    trk_kf: TrackerKF,
    num_hit: usize,
    unmatched_f_since_l: usize,
    iou3d_history: Vec<f32>,
}

pub struct AB3DMOT {
    max_age: usize,
    min_hit: usize,
    iou_threshold: f32,
    trackers: Vec<Tracker>, // tracker, unmatched_time
    frame_count: usize,
    // history_tracker_num: usize,
}

impl AB3DMOT {
    pub fn new(max_age: usize, min_hit: usize, iou_threshold: f32) -> Self {
        Self {
            max_age,
            min_hit,
            iou_threshold,
            trackers: Vec::<Tracker>::new(),
            frame_count: 0,
            // history_tracker_num:0,
        }
    }

    pub fn update(&mut self, frame: &IFrame) -> OFrame::Verbose /*有返回值，但不清楚类型*/ {
        let data_3d = frame.get_3d_infos();
        let (objects_type, remain_two): (Vec<_>, Vec<_>) = data_3d.into_iter().map(
            |x| (x.0, (x.1, x.2))
        ).unzip();
        let (dets_xyz, _scores): (Vec<_>, Vec<_>) = remain_two.into_iter().unzip();
        let dets_corner: Vec<_> = dets_xyz.iter().map(|x| x.to_CornerPoints()).collect();
        self.frame_count += 1;

        // 开始预测
        let tracker_predictions: Vec<_> = self.trackers.iter().map(
            |x| {
                let rst_ovecter = (&x.trk_kf as &TrackerKF).predict().state().clone();
                BBox3D::XYZLHWRotY(
                    rst_ovecter[0],
                    rst_ovecter[1],
                    rst_ovecter[2],
                    rst_ovecter[3],
                    rst_ovecter[4],
                    rst_ovecter[5],
                    rst_ovecter[6],
                ).to_CornerPoints()
            }
        ).collect();

        let (matched, unmatched_trks, unmatched_dets) = associate_detections_to_trackers(&tracker_predictions, &dets_corner, self.iou_threshold);
        
        for (trker_id, det_id, iou) in matched {
            let trker_mut_ref = &mut self.trackers[trker_id];
            trker_mut_ref.trk_kf.update(dets_xyz[det_id]);
            trker_mut_ref.iou3d_history.push(iou);
            trker_mut_ref.num_hit += 1;
        }

        // for trker_id in unmatched_trks.clone() {
        //     self.trackers[trker_id].unmatched_f_since_l += 1;
        // }

        for trker_id in {
            let mut unmatched_trks = unmatched_trks.clone();
            unmatched_trks.sort();
            unmatched_trks.reverse();
            unmatched_trks
        } {
            self.trackers[trker_id].unmatched_f_since_l += 1;
            self.trackers[trker_id].iou3d_history.push(0f32);
            if self.trackers[trker_id].unmatched_f_since_l > self.max_age {
                self.trackers.swap_remove(trker_id);
            }
        }  // 删除超出最大保存时间的 tracker

        let mut name_generator = Generator::default();

        for det_id in unmatched_dets {
            self.trackers.push(
                Tracker {
                    id: Uuid::new_v4(),
                    name: name_generator.next().unwrap(),
                    object_type: objects_type[det_id],
                    bbox_3d: dets_xyz[det_id],
                    trk_kf: TrackerKF::new(
                        dets_xyz[det_id],
                        [
                            10.0f32,
                            10.0f32,
                            10.0f32,
                            10.0f32,
                            10.0f32,
                            10.0f32,
                            10.0f32,
                            1000.0f32,
                            1000.0f32,
                            1000.0f32,
                    ]),
                    num_hit: 0,  // 第一次出现不认为是命中
                    unmatched_f_since_l: 0,  // 第一次出现也不认为是失配
                    iou3d_history: Vec::<f32>::new(),
                }
            )
        }  // 新的 tracker


        OFrame::Verbose {
            objects: self.trackers.iter().filter(|x| x.num_hit >= self.min_hit).map(
                |x| {
                    OObject::Verbose {
                        id: x.id,
                        name: x.name.clone(),
                        object_type: x.object_type,
                        bbox_3d: x.bbox_3d,
                        num_hit: x.num_hit,
                        unmatched_f_since_l: x.unmatched_f_since_l,
                        iou3d_history: x.iou3d_history.clone(),
                        score: *x.iou3d_history.last().unwrap(),
                    }
                }
            ).collect()
        }
    }
}