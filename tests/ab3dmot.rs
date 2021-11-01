use thundermot_with_rust::{data,
    ab3dmot::{AB3DMOT},
};

#[test]
fn ab3dmot_frame_test() -> () {
    let frame_seq = data::input::prepare_data().frames_seqs.swap_remove(0).frames;
    let mut ab3dmot = AB3DMOT::new(2, 1, 0.2);
    for (frame_num, each_frame) in frame_seq.iter().enumerate() {
        let rst = ab3dmot.update(each_frame);
        println!("frame_num: {}", frame_num);
        for each_tracked_object in rst.objects {
            println!("\tObject ID: {}", each_tracked_object.id);
            println!(
                "\t\tObject Name: {}\n\t\tObject BBox: {}\n\t\tHit Num: {}\n\t\tScore: {}",
                each_tracked_object.name,
                each_tracked_object.bbox_3d,
                each_tracked_object.num_hit,
                each_tracked_object.score,
            );
        };
    };
}