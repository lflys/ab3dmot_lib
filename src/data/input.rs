use std::io::Read;
use std::convert::TryFrom;

use super::BBox3D;
use super::BBox2D;
use super::ObjectType;

const DATA_FILE_PATHS: [&str; 1] = [
    "/home/pete/Projects/ROS2/thundermot_with_rust/data/KITTI/pointrcnn_Car_test/0000.txt"
];

pub struct Object {
    pub object_type: ObjectType,
    pub bbox_2d: BBox2D,
    pub bbox_2d_score: f32,
    pub bbox_3d: BBox3D::XYZLHWRotY,
    pub bbox_3d_alpha: f32,
}

pub struct Frame {
    pub objects: Vec<Object>,
}
impl Frame {
    pub fn get_3d_infos(&self) -> Vec<(ObjectType, BBox3D::XYZLHWRotY, f32)> {
        self.objects.iter().map(|x| (x.object_type, x.bbox_3d, x.bbox_3d_alpha/*.clone()*/)).collect()
    }
}

pub struct FrameSeq {
    pub frames: Vec<Frame>,
}

pub struct Data {
    pub frames_seqs: Vec<FrameSeq>,
}

pub fn prepare_data_raw_str() -> Vec<String> {
    let mut rst_data = Vec::<String>::new();
    for each_frame_seq_file in DATA_FILE_PATHS {
        let mut data_file = std::fs::File::open(std::path::Path::new(each_frame_seq_file)).unwrap();
        let mut data_raw_content = String::new();
        data_file.read_to_string(&mut data_raw_content).unwrap();
        let data_raw_content = data_raw_content.as_str();
        for each_line in data_raw_content.split('\n') {
            if each_line.trim().is_empty() {continue;}
            rst_data.push(each_line.to_string());
        }
    }

    rst_data
}

pub fn prepare_data() -> Data {
    let mut rst_data = Data {frames_seqs: Vec::<FrameSeq>::new()};
    for each_frame_seq_file in DATA_FILE_PATHS {

        let mut data_file = std::fs::File::open(std::path::Path::new(each_frame_seq_file)).unwrap();
        let mut data_raw_content = String::new();
        data_file.read_to_string(&mut data_raw_content).unwrap();
        let data_raw_content = data_raw_content.as_str();

        let mut frame_seq = FrameSeq {frames: Vec::<Frame>::new()};
        for each_line in data_raw_content.split('\n') {
            if each_line.trim().is_empty() {continue;}
            // let mut frame = Frame {objects: Vec::<Object>::new()};
            let (frame_num, object_type, x1,  y1,  x2,  y2,  score, h,   w,   l,   x,   y,   z,   rot_y, alpha):
                (usize,     u8,          f32, f32, f32, f32, f32,   f32, f32, f32, f32, f32, f32, f32,   f32);
            text_io::scan!(each_line.bytes() => "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}", frame_num, object_type, x1, y1, x2, y2, score, h, w, l, x, y, z, rot_y, alpha);

            // 在数据中加入缺失的帧
            for _ in frame_seq.frames.len()..=frame_num {
                frame_seq.frames.push(Frame{objects: Vec::<Object>::new()});
            }
            
            // 向相应帧中加入物体
            frame_seq.frames[frame_num].objects.push(Object {
                object_type: ObjectType::try_from(object_type).unwrap(),
                bbox_2d: BBox2D { x1, y1, x2, y2 },
                bbox_2d_score: score,
                bbox_3d: BBox3D::XYZLHWRotY(x, y, z, l, h, w, rot_y),
                bbox_3d_alpha: alpha
            });
        }
        rst_data.frames_seqs.push(frame_seq);
    }
    rst_data
}