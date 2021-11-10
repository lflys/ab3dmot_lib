use strum_macros::{Display, EnumString};

#[derive(Clone, Copy, num_enum::TryFromPrimitive, Display, EnumString)]
#[repr(u8)]
pub enum ObjectType {
    #[strum(ascii_case_insensitive)]
    Pedestrian = 1,
    #[strum(ascii_case_insensitive)]
    Car = 2,
    #[strum(ascii_case_insensitive)]
    Cyclist = 3,
}

pub struct BBox2D {
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
}


/*
  车顶O1._________\ z(汽车行进方向)
       /|         /
      / |
     /  |
 x |/_  |
       \|/
       y(竖直方向)

                                     (6)_________(7)
                                       /|       /|
                                      / |      / |
                                     /  |     /  |
                    ___          (5)/___|____/(4)|
                     |              |   |    |   |
                     |  ___         |(2)|____|___|(3)
                     h   /          |   /    |   /
                     |  w           |  /  O2.|__/_____\ z'
                     | /            | /    /|| /      /
                    _|/          (1)|/____/_||/(0)
                                         /  |
                                     x'|/_  |
                                           \|/
                                           y'

                                    |<--l--->|

    bbox 3d 的 (x, y, z) 是 O2 点在 O1XYZ 下的坐标

    由 XO1Y 绕 Y 轴 旋转到 X'O2Y' 的旋转角度为 theta，以从 z 轴转到 x 轴的方向为正
*/

pub mod BBox3D {
    use nalgebra as na;

    use derive_more::{Display};

    #[derive(Clone, Copy, Display)]
    #[display(fmt = "x: {}, y: {}, z: {}, l: {}, w: {}, h: {}, rot_y: {}", "_0", "_1", "_2", "_3", "_4", "_5", "_6", )]
    pub struct XYZLHWRotY(pub f32, pub f32, pub f32, pub f32, pub f32, pub f32, pub f32, );
    impl XYZLHWRotY {
        pub fn to_CornerPoints(&self) -> CornerPoints {
            let Self(x, y, z, l, h, w, rot_y) = self;
            let iso = na::Isometry3::new(
                na::Vector3::new(-x, -y, -z),
                na::Vector3::y() * (*rot_y) * core::f32::consts::FRAC_PI_2,
            );
            CornerPoints(
                // 变换矩阵 乘以 0~7 这 8 个边角点在 02X'Y'Z' 下的坐标
                iso * na::Point3::<f32>::new( l/2f32,  0f32,  w/2f32),
                iso * na::Point3::<f32>::new( l/2f32,  0f32, -w/2f32),
                iso * na::Point3::<f32>::new(-l/2f32,  0f32, -w/2f32),
                iso * na::Point3::<f32>::new(-l/2f32,  0f32,  w/2f32),
                iso * na::Point3::<f32>::new( l/2f32, -h,     w/2f32),
                iso * na::Point3::<f32>::new( l/2f32, -h,    -w/2f32),
                iso * na::Point3::<f32>::new(-l/2f32, -h,    -w/2f32),
                iso * na::Point3::<f32>::new(-l/2f32, -h,     w/2f32)
            )
        }
    }

    #[derive(Clone, Copy, Debug)]
    pub struct CornerPoints(
        pub na::Point3<f32>,
        pub na::Point3<f32>,
        pub na::Point3<f32>,
        pub na::Point3<f32>,
        pub na::Point3<f32>,
        pub na::Point3<f32>,
        pub na::Point3<f32>,
        pub na::Point3<f32>,
    );
}

pub mod input;

pub mod output;