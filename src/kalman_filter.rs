use core::{f64::consts::{PI}, panic};
use std::{sync::{Arc, Mutex}, f64::consts::FRAC_2_PI};

use nalgebra::{Const, OMatrix, OVector, RealField, dimension::{U7, U10}, one, zero, ComplexField};
use adskalman::{KalmanFilterNoControl, TransitionModelLinearNoControl, ObservationModel, StateAndCovariance};
use lazy_static::lazy_static;

use mot_data::{BBox3D, };
use once_cell::sync::OnceCell;

/// 带有自定类型 R 的匀速运动模型
#[derive(Clone, Copy, Debug)]
pub struct ConstantVelocity3DModel<R>
where R: RealField,
{
    /// ### Kalman 状态转移矩阵 F_k
    ///
    /// ```plain text
    /// [1, 0, 0, 0, 0, 0, 0, t_k, 0  , 0  , ]   [x]
    /// [0, 1, 0, 0, 0, 0, 0, 0  , t_k, 0  , ]   [y]
    /// [0, 0, 1, 0, 0, 0, 0, 0  , 0  , t_k, ]   [z]
    /// [0, 0, 0, 1, 0, 0, 0, 0  , 0  , 0  , ]   [w]
    /// [0, 0, 0, 0, 1, 0, 0, 0  , 0  , 0  , ]   [h]
    /// [0, 0, 0, 0, 0, 1, 0, 0  , 0  , 0  , ] * [l]
    /// [0, 0, 0, 0, 0, 0, 1, 0  , 0  , 0  , ]   [rot_y]
    /// [0, 0, 0, 0, 0, 0, 0, 1  , 0  , 0  , ]   [v_x]
    /// [0, 0, 0, 0, 0, 0, 0, 0  , 1  , 0  , ]   [v_y]
    /// [0, 0, 0, 0, 0, 0, 0, 0  , 0  , 1  , ]   [v_z]
    /// ^                                        ^
    /// F_k                                    * state_k
    /// ```
    /// 1. v_x、v_y、v_z 分别代表自定单位下 x、y、z 轴上的速度
    /// 2. state_k.x = state_k-1.x + t_k * state_k-1.v_k
    /// 3. 所以可以看出这个转换模型在每个时间 k 都是会变化的，需给出距上次的时间间隔 t_k
    transition_model_f: OMatrix<R, U10, U10>,
    transition_model_f_transpose: OMatrix<R, U10, U10>,
    /// ### Kalman 模型处理误差 Q
    transition_noise_covariance_q: OMatrix<R, U10, U10>,
}
impl<R> ConstantVelocity3DModel<R>
where R: RealField + Copy,
{
    /// 传入 Q 的对角元素和据上一次的时间间隔
    pub fn new(transition_noise_var_q: [R; 10], delta_t: R) -> Self {
        let mut transition_model_f = OMatrix::identity_generic(Const::<10>, Const::<10>);
        // 设置更新的间隔时间
        transition_model_f[(0, 7)] = delta_t; transition_model_f[(1, 8)] = delta_t; transition_model_f[(2, 9)] = delta_t;

        let mut transition_noise_covariance_q = OMatrix::identity_generic(Const::<10>, Const::<10>);
        for (idx, var) in (0..).zip(transition_noise_var_q) {
            transition_noise_covariance_q[(idx, idx)] = var;
        }

        Self {
            transition_model_f,
            transition_model_f_transpose: transition_model_f.transpose(),
            transition_noise_covariance_q
        }
    }

    /// 更新间隔时间
    pub fn set_delta_time(&mut self, delta_t: R) -> &mut Self {
        // 更新间隔时间
        self.transition_model_f[(0, 7)] = delta_t;
        self.transition_model_f[(1, 8)] = delta_t;
        self.transition_model_f[(2, 9)] = delta_t;
        self.transition_model_f_transpose[(7, 0)] = delta_t;
        self.transition_model_f_transpose[(8, 1)] = delta_t;
        self.transition_model_f_transpose[(9, 2)] = delta_t;
        self
    }
}
impl<R> TransitionModelLinearNoControl<R, U10> for ConstantVelocity3DModel<R>
where
    R: RealField,
{
    fn F(&self) -> &OMatrix<R, U10, U10> {
        &self.transition_model_f
    }

    fn FT(&self) -> &OMatrix<R, U10, U10> {
        &self.transition_model_f_transpose
    }

    fn Q(&self) -> &OMatrix<R, U10, U10> {
        &self.transition_noise_covariance_q
    }
}

#[derive(Debug)]
pub struct PositionObservation3DModel<R>
where R: RealField
{
    pub observation_matrix_h: OMatrix<R, U7, U10>,
    pub observation_matrix_h_transpose: OMatrix<R, U10, U7>,
    pub observation_noise_covariance_r: OMatrix<R, U7, U7>,
}
impl<R> PositionObservation3DModel<R>
where R: RealField + Copy
{
    pub fn new(observation_noise_var_r: [R; 7]) -> Self {
        // let (r_zero, r_one) = (zero::<R>(), one::<R>());
        let observation_matrix_h = OMatrix::identity_generic(Const::<7>, Const::<10>);

        let mut observation_noise_covariance_r = OMatrix::identity_generic(Const::<7>, Const::<7>);
        for (idx, var) in (0..).zip(observation_noise_var_r) {
            observation_noise_covariance_r[(idx, idx)] = var;
        }

        Self {
            observation_matrix_h,
            observation_matrix_h_transpose: observation_matrix_h.transpose(),
            observation_noise_covariance_r
        }
    }
}
impl<R> ObservationModel<R, U10, U7> for PositionObservation3DModel<R>
where R: RealField,
{
    fn H(&self) -> &OMatrix<R, U7, U10> {
        &self.observation_matrix_h
    }
    fn HT(&self) -> &OMatrix<R, U10, U7> {
        &self.observation_matrix_h_transpose
    }
    fn R(&self) -> &OMatrix<R, U7, U7> {
        &self.observation_noise_covariance_r
    }
}

/// 从此开始，定义模型内部使用的数据类型
type TypeUnderModel = f32;

/*
 * 这里设置 MOTION_MODEL、OBSERVATION_MODEL 为两个静态类型
 * 1. 首先 MOTION_MODEL 和 OBSERVATION_MODEL 在所有 Tracklet 的更新中都需要使用
 * 2. 为每个 Tracklet 单独设置一个 MOTION_MODEL 和 OBSERVATION_MODEL 有大量浪费，因为它们的内容都一样
 * 3. 不能设置为 static，由于 MOTION_MODEL、OBSERVATION_MODEL 的 new 函数不是 static function
 * 4. MOTION_MODEL 需要设置为 Mutex<...>，因为 lazy_static! 内如果需要 mutable 则需要是 thread safe 的，这是 lazy_static 宏的要求
 *    Mutex 保证了对其中的操作是互斥的
 *    
 * todo:
 * 这里的方差设置需要通过接口的形式释放出来，而不是作为硬编码写死
 */
// lazy_static! {
//     static ref MOTION_MODEL: Mutex<ConstantVelocity3DModel<TypeUnderModel>> = Mutex::new(ConstantVelocity3DModel::new(
//         [
//             // 这里的对于状态转移模型处理的方差要看问题的规模的单位，所以不宜定死
//             0.05 as TypeUnderModel,
//             0.05 as TypeUnderModel,
//             0.05 as TypeUnderModel,
//             0.05 as TypeUnderModel,
//             0.05 as TypeUnderModel,
//             0.05 as TypeUnderModel,
//             0.05 as TypeUnderModel,
//             1e-10 as TypeUnderModel,
//             1e-10 as TypeUnderModel,
//             1e-10 as TypeUnderModel,
//         ],
//         1 as TypeUnderModel,
//     ));
//     static ref OBSERVATION_MODEL: PositionObservation3DModel<TypeUnderModel> = PositionObservation3DModel::new([0.01 as TypeUnderModel; 7]);
// }
static MOTION_MODEL: OnceCell<Mutex<ConstantVelocity3DModel<TypeUnderModel>>> = OnceCell::new();
static OBSERVATION_MODEL: OnceCell<PositionObservation3DModel<TypeUnderModel>> = OnceCell::new();


#[derive(Clone)]
pub struct TrackerKF {
    previous_est: StateAndCovariance<TypeUnderModel, U10>,
    observation: OVector<TypeUnderModel, U7>,
}
impl TrackerKF {
    pub fn set_convs(motion_model_process_conv: [TypeUnderModel; 10], observation_conv: [TypeUnderModel; 7]) {
        MOTION_MODEL.set(Mutex::new(ConstantVelocity3DModel::new(motion_model_process_conv, 1 as TypeUnderModel))).expect("MOTION_MODEL can only be set once.");
        OBSERVATION_MODEL.set(PositionObservation3DModel::new(observation_conv)).expect("OBSERVATION_MODEL can only be set once.");
    }

    /// 将弧度制下任意一角度转为 [0, 2π]
    #[inline]
    fn wrap_2_0_2_2pi(angle: TypeUnderModel) -> TypeUnderModel {
        let floor = (angle / ((2f64*PI) as TypeUnderModel)).floor();  // angle - floor*2π ∈ [0, 2π]
        angle - floor * ((2f64*PI) as TypeUnderModel)
    }

    /// 将弧度制下任意一角度转为 [-π, π]
    #[inline]
    fn wrap_2_minus_pi_2_pi(angle: TypeUnderModel) -> TypeUnderModel {
        Self::wrap_2_0_2_2pi(angle + PI as TypeUnderModel) - PI as TypeUnderModel
    }
    
    /// 在弧度制下将一角度转化为所在的直线的角度 [-π/2, π/2] 内
    #[inline]
    fn wrap_2_minus_pi_2_2_pi_2(angle: TypeUnderModel) -> TypeUnderModel {
        let angle = Self::wrap_2_minus_pi_2_pi(angle);
        if angle.abs() >= FRAC_2_PI as TypeUnderModel {
            Self::wrap_2_minus_pi_2_pi(angle + PI as TypeUnderModel)
        } else {angle}
    }

    /// 这里要给出一个新的 tracklet 第一次的 bbox，和第一次观察的方差
    pub fn new(bbox3d: BBox3D::XYZWHLRotY, initial_estimation_noise_var: [TypeUnderModel; 10]) -> Self {
        let BBox3D::XYZWHLRotY(x, y, z, l, h, w, rot_y) = bbox3d;
        
        // 初始状态，速度的初始状态置为零
        let init_state_x = OVector::<TypeUnderModel, U10>::from_iterator([
            x as TypeUnderModel,
            y as TypeUnderModel,
            z as TypeUnderModel,
            l as TypeUnderModel,
            h as TypeUnderModel,
            w as TypeUnderModel,
            rot_y as TypeUnderModel,
            0 as TypeUnderModel,  // vx
            0 as TypeUnderModel,  // vy
            0 as TypeUnderModel,  // vz
        ]);

        // 初始方差
        let mut initial_covariance_p = OMatrix::<TypeUnderModel, U10, U10>::identity();
        for (idx, var) in (0..).zip(initial_estimation_noise_var) {
            initial_covariance_p[(idx, idx)] = var;
        }

        // 上一次的估计值
        let previous_est = StateAndCovariance::new(init_state_x, initial_covariance_p);
        // 最近一次观测值
        // 这里需要是第一次的状态吗？
        let observation = OVector::<TypeUnderModel, U7>::zeros();

        Self {
            previous_est,
            observation,
        }
    }
    
    // 当前的状态就是
    pub fn get_latest_observation(&self) -> BBox3D::XYZWHLRotY {
        BBox3D::XYZWHLRotY(
            self.observation[0],
            self.observation[1],
            self.observation[2],
            self.observation[3],
            self.observation[4],
            self.observation[5],
            self.observation[6],
        )
    }
    
    pub fn predict(&self, delta_t: TypeUnderModel) -> StateAndCovariance<TypeUnderModel, U10> {
        MOTION_MODEL.get().expect("MOTION_MODEL hasn't been setted.").lock().unwrap().set_delta_time(delta_t).predict(&self.previous_est)
    }

    pub fn update(&mut self, BBox3D::XYZWHLRotY(x, y, z, w, h, l, rot_y): BBox3D::XYZWHLRotY, delta_t: TypeUnderModel) {
        let mut previous_est_angle = Self::wrap_2_minus_pi_2_pi(self.previous_est.state()[6]);  // 把上一状态中的「角度」设为 -π 到 +π

        let mut angle_obs = Self::wrap_2_minus_pi_2_pi(rot_y);  // 把到来的检测框的「角度」设为 -π 到 +π

        /*
            下面将状态中的 θ_p (上一状态中的「角度」) 和 θ_o （到来的新的观测值检测框的「角度」）之间做转化，
            目的是将最后的 θ_p 和 θ_o 之间的角度差缩小至 [0, π/2] 之间：
            
            对于一个 3d 检测框来说，它的转动角度使用 [-π/2, π/2] 就足够表示了，但是如果使用 [-π/2, π/2] 表示就会出现一个问题：
            有一个物体原来的转动角度很接近 -π/2 然后随后往负角度方向转动了一小点，但是在表示上由于超过了范围，等价的变为了靠近 π/2 的一个角度，
            这在 kalman filter 中是一个很大的改变，对于随后的旋转量估计会变得不够精准，而如果采用 [-π, π] 的表示法的话，可以使用角度的等价关系，将前后两个角度
            的变化量等价转换至 [0, π/2] 内。
            
            1. 若两个角度差本来就在 [0, π/2] 之间的话，不变
            2. 如果角度差在 [π/2, 2π] 之间：
                将其中任意一个角反转 180° 再设为 -π 至 +π 间

         */

        {
            let angle_dis = (previous_est_angle - angle_obs).abs();
            // 首先不论谁减谁得到的结果都是两个角度的差值
            if angle_dis > (PI/2f64) as TypeUnderModel && angle_dis < PI as TypeUnderModel {
                /*
                角度差在 [π/2, π] 之间时
                1. 两个角度同为正，只能将最大的角转 180 度
                .         .:`
                `:. .:`          +
                --------------------
                                -

                2. 两个角度同为负，只能将最小的角转 180 度
                3. 两个角度一正一负，任意角度可转 180 度
                */
                if (previous_est_angle*angle_obs) > 0f64 as TypeUnderModel {  // 两角度同号
                    if previous_est_angle > 0f64 as TypeUnderModel {  // 两角度同正
                        if previous_est_angle > angle_obs {  // 如果 θ_p 较大
                            previous_est_angle = Self::wrap_2_minus_pi_2_pi(previous_est_angle+PI as TypeUnderModel);
                        } else {  // 如果 θ_o 较大
                            angle_obs = Self::wrap_2_minus_pi_2_pi(angle_obs+PI as TypeUnderModel);
                        }
                    } else {  // 两角度同负
                        if previous_est_angle < angle_obs {  // 如果 θ_p 较小
                            previous_est_angle = Self::wrap_2_minus_pi_2_pi(previous_est_angle+PI as TypeUnderModel);
                        } else {  // 如果 θ_o 较小
                            angle_obs = Self::wrap_2_minus_pi_2_pi(angle_obs+PI as TypeUnderModel);
                        }
                    }
                } else {  // 两角度不同号
                    previous_est_angle = Self::wrap_2_minus_pi_2_pi(previous_est_angle+PI as TypeUnderModel);
                }
            } else if angle_dis > PI as TypeUnderModel && angle_dis < (PI*3f64/2f64) as TypeUnderModel {
                // 在 [π, 3π/2] 之间，必定是一正一负了，随便选择一个转动 180 度
                previous_est_angle = Self::wrap_2_minus_pi_2_pi(previous_est_angle+PI as TypeUnderModel);
            } else {
                // 在 [3π/2, 2π] 之间
                // 两者均转动 180 度
                previous_est_angle = Self::wrap_2_minus_pi_2_pi(previous_est_angle+PI as TypeUnderModel);
                angle_obs = Self::wrap_2_minus_pi_2_pi(angle_obs+PI as TypeUnderModel);
            }
        }

        let mut state_revision = *self.previous_est.state();
        state_revision[6] = previous_est_angle;

        // 这里对 previous_est 是有修改的，再这之前很可能已经读取过 previous_est 了
        self.previous_est = StateAndCovariance::new(state_revision, *self.previous_est.covariance());
        
        self.observation = OVector::<TypeUnderModel, U7>::from_iterator([
            x as TypeUnderModel,
            y as TypeUnderModel,
            z as TypeUnderModel,
            w as TypeUnderModel,
            h as TypeUnderModel,
            l as TypeUnderModel,
            angle_obs as TypeUnderModel,
        ]);

        let motion_model = *MOTION_MODEL.get().expect("MOTION_MODEL hasn't been setted.").lock().unwrap().set_delta_time(delta_t);
        let kf = KalmanFilterNoControl::new(&motion_model, &*OBSERVATION_MODEL.get().expect("OBSERVATION_MODEL hasn't been setted."));
        self.previous_est = match kf.step(&self.previous_est, &self.observation) {
            Ok(x) => x,
            Err(e) => {
                println!("{}", e.to_string());
                panic!();
            }
        };
        let state = self.previous_est.state();
        let (vx, vy, vz) = (state[7], state[8], state[9]);
        let if_break_here = vx > 0.001 || vy > 0.001 || vz > 0.001;
        let a = 0;
    }
}