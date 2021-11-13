//! # 数据输出接口

/// 输出物品接口模块
pub mod Object {
    /// 输出物体的精简信息
    pub type Concise = super::inner::_Object;
    /// 输出物体的详细信息
    pub type Verbose = super::inner::_ObjectVerbose;
}

/// 输出帧接口模块
pub mod Frame {
    /// 输出帧的精简信息
    pub type Concise = super::inner::_Frame;
    /// 输出帧的详细信息
    pub type Verbose = super::inner::_FrameVerbose;
}

mod inner{
    use super::super::{ObjectType, BBox2D, BBox3D};

    use derive_more::{Display};

    use uuid::Uuid;

    /// bbox_2d 和 bbox_3d 不一定同时存在
    pub struct _Object {
        /// Uuid V4
        pub id: Uuid,
        pub object_type: ObjectType,
        pub bbox_2d: Option<BBox2D>,
        pub bbox_3d: Option<BBox3D::XYZWHLRotY>,
        pub score: f32,
    }

    /// bbox_2d 和 bbox_3d 不一定同时存在
    pub struct _ObjectVerbose {
        pub id: Uuid,
        pub name: String,
        pub object_type: ObjectType,
        pub bbox_2d: Option<BBox2D>,
        pub bbox_3d: Option<BBox3D::XYZWHLRotY>,
        pub num_hit: usize,
        pub unmatched_f_since_l: usize,
        pub iou3d_history: Vec<f32>,
        pub score: f32,  // iou3d
    }

    pub struct _Frame {
        pub objects: Vec<_Object>,
    }

    pub struct _FrameVerbose {
        pub objects: Vec<_ObjectVerbose>,
    }

    pub struct _FrameSeq {
        pub frames: Vec<_Frame>,
    }

    pub struct _FrameSeqVerbose {
        pub frames: Vec<_FrameVerbose>,
    }
}