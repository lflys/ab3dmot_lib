use std::f32::consts::FRAC_PI_6;

use ab3dmot::{AB3DMOT, iou3d};
use itertools::Itertools;
use mot_data::BBox3D;

use super::prepare_data;

#[test]
fn ab3dmot_frame_test() -> () {
    let frame_seq = prepare_data().frames_seqs.swap_remove(0).frames;
    let mut ab3dmot = AB3DMOT::new(2, 1, 0.2);
    for (frame_num, each_frame) in frame_seq.iter().map(|x| Some(x)).intersperse(None).enumerate() {
        if let Some(frame) = each_frame {
            let rst = ab3dmot.update(frame);
            println!("frame_num: {}", frame_num);
            for each_tracked_object in rst.objects {
                println!("\tObject ID: {}", each_tracked_object.id);
                println!(
                    "\t\tObject Name: {}\n\t\tObject BBox: {}\n\t\tHit Num: {}\n\t\tScore: {}",
                    each_tracked_object.name,
                    each_tracked_object.bbox_3d.unwrap(),
                    each_tracked_object.num_hit,
                    each_tracked_object.score,
                );
            };
        } else {
            let rst = ab3dmot.update(&1usize);
            println!("frame_num: {}", frame_num);
            for each_tracked_object in rst.objects {
                println!("\tObject ID: {}", each_tracked_object.id);
                println!(
                    "\t\tObject Name: {}\n\t\tObject BBox: {}\n\t\tHit Num: {}\n\t\tScore: {}",
                    each_tracked_object.name,
                    each_tracked_object.bbox_3d.unwrap(),
                    each_tracked_object.num_hit,
                    each_tracked_object.score,
                );
            };
        }
    };
}

#[test]
fn iou3d_test() -> () {
    let (bbox_1, bbox_2) = (
        BBox3D::XYZWHLRotY(1f32, 0f32, 2f32, 2f32, 1f32, 2f32, 0f32),
        BBox3D::XYZWHLRotY(1f32, 0f32, 2f32, 2f32, 1f32, 2f32, FRAC_PI_6),
    );
    println!("{:?}", iou3d(
        &bbox_1.to_CornerPoints(),
        &bbox_2.to_CornerPoints(),
    ));
}