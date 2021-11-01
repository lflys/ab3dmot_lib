pub mod thundermot {
    pub struct TrackedObject {
        pub age_count: i32,
        pub hit_count: i32,
        pub motion_state: MotionState,
        // pub latest: autoware_msgs::DetectedObject,
    }

    pub struct TrackingStateStore {
        pub cur_frame: String,
    }
    impl TrackingStateStore {
        pub fn new(/*ros::NodeHandle*/) -> Self {Self{}}
    }
}