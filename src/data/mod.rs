//! # 数据接口

use strum_macros::{Display, EnumString};  // Display 用于对 enum 进行显示，而 EnumString 则用于从 String 构建 enum

#[derive(Clone, Copy, num_enum::TryFromPrimitive, Display, EnumString)]  // 这里的 num_enum::TryFromPrimitive 用于从 int 构建 enum
#[repr(u8)]  // num_enum::TryFromPrimive 中需要
/// 对象类型：
pub enum ObjectType {
    #[strum(ascii_case_insensitive)]
    /// 行人
    Pedestrian = 1,
    #[strum(ascii_case_insensitive)]
    /// 车辆
    Car = 2,
    #[strum(ascii_case_insensitive)]
    /// 骑行者
    Cyclist = 3,
}

/// 2d 检测框
pub struct BBox2D {
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
}


pub mod BBox3D {
//! # 3d 检测框
//! ## 针对坐标系的说明
//! 
//! ```txt
//!  车顶O1._________\ z(汽车行进方向)
//!       /|         /
//!      / |
//!     /  |
//! x |/_  |
//!       \|/
//!       y(竖直方向)
//!
//!                                     (6)_________(7)
//!                                       /|       /|
//!                                      / |      / |
//!                                     /  |     /  |
//!                    ___          (5)/___|____/(4)|
//!                     |              |   |    |   |
//!                     |  ___         |(2)|____|___|(3)
//!                     h   /          |   /    |   /
//!                     |  w           |  /  O2.|__/_____\ z'
//!                     | /            | /    /|| /      /
//!                    _|/          (1)|/____/_||/(0)
//!                                         /  |
//!                                     x'|/_  |
//!                                           \|/
//!                                           y'
//!
//!                                    |<--l--->|
//!
//!    bbox 3d 的 (x, y, z) 是 O2 点在 O1XYZ 下的坐标
//!
//!    由 XO1Y 绕 Y 轴 旋转到 X'O2Y' 的旋转角度为 theta，以从 z 轴转到 x 轴的方向为正
//! ```
    use nalgebra as na;

    use derive_more::{Display};

    #[derive(Clone, Copy, Display)]
    #[display(fmt = "x: {}, y: {}, z: {}, w: {}, h: {}, l: {}, rot_y: {}", "_0", "_1", "_2", "_3", "_4", "_5", "_6", )]
    /// 这里谁是长，谁是宽不重要，重要的是在物体自身的坐标系经过旋转到与车辆所在的坐标系平行之后，哪两个值分别对应是沿车辆 x, z 轴方向上物体的长度
    /// 这里的 LHW 分别对应的是经旋转后 X', Y, Z' 轴上的的长度
    pub struct XYZWHLRotY(pub f32, pub f32, pub f32, pub f32, pub f32, pub f32, pub f32, );
    impl XYZWHLRotY {
        /// 将 XYZWHLRotY 转为八个点的表示方式
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
    /// 八个点表示法
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