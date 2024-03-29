use std::{convert::TryFrom, str::FromStr};
use super::prepare_data;

#[test]
fn input_data_get3d() -> () {
    let data_1 = prepare_data();
    for each_frame_seq in data_1.frames_seqs {
        for each_frame in each_frame_seq.frames {
            for (object_type, xyz, score) in each_frame.get_3d_infos() {
                println!("Object Type: {}\txyz: {}\tscore:{}", object_type, xyz, score);
            }
        }
    }
}

#[test]
fn convert_str_2_type_enum() -> () {
    println!("{}", mot_data::ObjectType::from_str("car").unwrap());
    println!("{}", mot_data::ObjectType::try_from(2).unwrap());
}