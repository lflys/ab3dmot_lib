pub mod Object {
    pub type Concise = super::inner::_Object;
    pub type Verbose = super::inner::_ObjectVerbose;
}

pub mod Frame {
    pub type Concise = super::inner::_Frame;
    pub type Verbose = super::inner::_FrameVerbose;
}

mod inner{
    use super::super::{ObjectType, BBox3D};

    use derive_more::{Display};

    use uuid::Uuid;

    pub struct _Object {
        pub id: Uuid,
        pub object_type: ObjectType,
        pub bbox_3d: BBox3D::XYZLHWRotY,
        pub score: f32,
    }

    pub struct _ObjectVerbose {
        pub id: Uuid,
        pub name: String,
        pub object_type: ObjectType,
        pub bbox_3d: BBox3D::XYZLHWRotY,
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
}