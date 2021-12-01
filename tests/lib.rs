use std::io::Read;
use std::convert::TryFrom;

use mot_data::{BBox2D, BBox3D, ObjectType, input::{Data, Frame, FrameSeq, Object}};

/// 这里是需要使用的内部数据集的文件地址，其规范是：
/// 1. array 中的每一个 &str 都代表一个连续采集的多帧数据
/// 2. 可以有多个数据
/// 
const DATA_FILE_PATHS: [&str; 1] = [
    "/home/pete/Projects/ab3dmot-rust/ab3dmot/data/KITTI/pointrcnn_Car_test/0000.txt"
];
// 本来这里是可以直接用 include_str! 来写入 binary 的，但是发现它太慢了，就没有用这个方法了

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
                frame_seq.frames.push(Frame{objects: Vec::<Object>::new(), delta_t:1usize});
            }
            
            // 向相应帧中加入物体
            frame_seq.frames[frame_num].objects.push(Object {
                object_type: ObjectType::try_from(object_type).unwrap(),
                bbox_2d: Some(BBox2D { x1, y1, x2, y2 }),
                bbox_2d_score: Some(score),
                bbox_3d: Some(BBox3D::XYZWHLRotY(x, y, z, l, h, w, rot_y)),
                bbox_3d_alpha: Some(alpha),
            });
        }
        rst_data.frames_seqs.push(frame_seq);
    }
    rst_data
}

mod data_test;

mod ab3dmot_test;