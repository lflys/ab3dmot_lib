use ab3dmot::{AB3DMOT, };
use itertools::Itertools;

use super::prepare_data;

#[test]
fn ab3dmot_frame_test() -> () {
    let frame_seq = prepare_data().frames_seqs.swap_remove(0).frames;
    let mut ab3dmot = AB3DMOT::new(2, 1, 0.2);
    for (frame_num, each_frame) in frame_seq.iter().map(|x| Some(x)).intersperse(None).enumerate() {
        let rst = ab3dmot.update(each_frame);
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
    };
}