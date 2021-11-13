//! # 数据输入接口

use super::BBox3D;
use super::BBox2D;
use super::ObjectType;

/// 数据输入中用于封装单个识别的对象
/// 可以不需要同时有 bbox_2d 和 bbox_3d
pub struct Object {
    /// 对象类型：车，行人，骑车人……
    pub object_type: ObjectType,
    /// 对象 2d 检测框
    pub bbox_2d: Option<BBox2D>,
    /// 对象 2d 检测框分数
    pub bbox_2d_score: Option<f32>,
    /// 以物体中心点坐标和对应轴方向上的长度
    pub bbox_3d: Option<BBox3D::XYZWHLRotY>,
    /// 对象 3d 检测框分数
    pub bbox_3d_alpha: Option<f32>,
}

/// 包含多个识别物体的一帧
pub struct Frame {
    pub objects: Vec<Object>,
}
impl Frame {
    /// 从这一帧中抽取所有带 3d 信息的物体
    pub fn get_3d_infos(&self) -> Vec<(ObjectType, BBox3D::XYZWHLRotY, f32)> {
        self.objects.iter().map(
            |x| {
                if let Object { object_type, bbox_3d: Some(x_y_z_w_h_l_roty), bbox_3d_alpha: Some(score), ..} = x {
                    Some((*object_type, *x_y_z_w_h_l_roty, *score))
                } else {
                    None
                }
            }
        ).filter(
            |x| matches!(x, Some(_))
        ).map(
            |x| x.unwrap()
        ).collect()
    }
}

/// 带有一系列连续帧的序列
pub struct FrameSeq {
    pub frames: Vec<Frame>,
}

/// 带有多段帧序列的数据
pub struct Data {
    pub frames_seqs: Vec<FrameSeq>,
}
