use core::{f64::consts::{PI}, panic};

use nalgebra::{Const, OMatrix, OVector, RealField, dimension::{U7, U10}, one, zero};
use adskalman::{KalmanFilterNoControl, TransitionModelLinearNoControl, ObservationModel, StateAndCovariance};
use lazy_static::lazy_static;

use crate::data::{BBox3D, };

pub struct ConstantVelocity3DModel<R>
where R: RealField,
{
    transition_model_f: OMatrix<R, U10, U10>,
    transition_model_f_transpose: OMatrix<R, U10, U10>,
    transition_noise_covariance_q: OMatrix<R, U10, U10>,
}
impl<R> ConstantVelocity3DModel<R>
where R: RealField + Copy,
{
    pub fn new(transition_noise_var_q: [R; 10]) -> Self {
        let (r_zero, r_one) = (zero::<R>(), one::<R>());
        /*
        let (r_two, r_three, r_four, r_five, r_six, r_seven, r_eight, r_nine, r_ten) = (
            r_one + r_one,
            r_one + r_one + r_one,
            r_one + r_one + r_one + r_one,
            r_one + r_one + r_one + r_one + r_one,
            r_one + r_one + r_one + r_one + r_one + r_one,
            r_one + r_one + r_one + r_one + r_one + r_one + r_one,
            r_one + r_one + r_one + r_one + r_one + r_one + r_one + r_one,
            r_one + r_one + r_one + r_one + r_one + r_one + r_one + r_one + r_one,
            r_one + r_one + r_one + r_one + r_one + r_one + r_one + r_one + r_one + r_one,
        );
        */
        let mut transition_model_f = OMatrix::identity_generic(Const::<10>, Const::<10>);
        transition_model_f[(0, 7)] = r_one; transition_model_f[(1, 8)] = r_one; transition_model_f[(2, 9)] = r_one;

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

type TypeUnderModel = f32;

lazy_static! {
    static ref MOTION_MODEL: ConstantVelocity3DModel<TypeUnderModel> = ConstantVelocity3DModel::new([
        0 as TypeUnderModel,
        0 as TypeUnderModel,
        0 as TypeUnderModel,
        0 as TypeUnderModel,
        0 as TypeUnderModel,
        0 as TypeUnderModel,
        0 as TypeUnderModel,
        0.1 as TypeUnderModel,
        0.1 as TypeUnderModel,
        0.1 as TypeUnderModel,
    ]);
    static ref OBSERVATION_MODEL: PositionObservation3DModel<TypeUnderModel> = PositionObservation3DModel::new([0.1 as TypeUnderModel; 7]);
}


pub struct TrackerKF {
    // id: usize,
    // hit_streak: usize,
    // info: f32,  // unknown

    ////////////////////
    // kf: KalmanFilterNoControl<'a, TypeUnderModel, U10, U7>,
    previous_est: StateAndCovariance<TypeUnderModel, U10>,
    observation: OVector<TypeUnderModel, U7>,
    // history: Vec<()>,  // unknown
    // hits: usize,
    // age: usize,

    ////////////////////
    // motion_model: ConstantVelocity3DModel<TypeUnderModel>,
    // observation_model: PositionObservation3DModel<TypeUnderModel>
}
impl TrackerKF {
    // const HIT_STREAK: usize = 2;
    // const MOTION_MODEL_LAYOUT:Layout = Layout::new::<ConstantVelocity3DModel<TypeUnderModel>>();
    // const OBSERVATION_MODEL_LAYOUT:Layout = Layout::new::<PositionObservation3DModel<TypeUnderModel>>();

    #[inline]
    fn wrap_2_0_2_2pi(angle: TypeUnderModel) -> TypeUnderModel {
        let floor = (angle / ((2f64*PI) as TypeUnderModel)).floor();
        angle - floor * ((2f64*PI) as TypeUnderModel)
    }

    #[inline]
    fn wrap_2_minus_pi_2_pi(angle: TypeUnderModel) -> TypeUnderModel {
        Self::wrap_2_0_2_2pi(angle + PI as TypeUnderModel) - PI as TypeUnderModel
    }

    pub fn new(bbox3d: BBox3D::XYZWHLRotY, initial_estimation_noise_var: [TypeUnderModel; 10]) -> Self {
        let BBox3D::XYZWHLRotY(x, y, z, l, h, w, rot_y) = bbox3d;
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

        let mut initial_covariance_p = OMatrix::<TypeUnderModel, U10, U10>::identity();
        for (idx, var) in (0..).zip(initial_estimation_noise_var) {
            initial_covariance_p[(idx, idx)] = var;
        }

        let previous_est = StateAndCovariance::new(init_state_x, initial_covariance_p);
        let observation = OVector::<TypeUnderModel, U7>::zeros();

        Self {
            previous_est,
            observation,
            // history: Vec::<()>::new(),
        }
    }
    
    pub fn predict(&self) -> StateAndCovariance<TypeUnderModel, U10> {
        MOTION_MODEL.predict(&self.previous_est)
    }

    pub fn update(&mut self, BBox3D::XYZWHLRotY(x, y, z, l, h, w, rot_y): BBox3D::XYZWHLRotY) {
        let mut previous_est_angle = Self::wrap_2_minus_pi_2_pi(self.previous_est.state()[6]);  // 把上一状态中的「角度」设为 -π 到 +π

        let mut angle_obs = Self::wrap_2_minus_pi_2_pi(rot_y);  // 把到来的检测框的「角度」设为 -π 到 +π

        /*
            下面将状态中的 θ_p (上一状态中的「角度」) 和 θ_o （到来的新的观测值检测框的「角度」）之间做转化，
            目的是将最后的 θ_p 和 θ_o 之间的角度差缩小至 [0, π/2] 之间：

            1. 若两个角度本来就在 [0, π/2] 之间的话，不变
            2. 如果角度差在 [π/2, 2π] 之间：
                将其中任意一个角反转 180° 再设为 -π 至 +π 间

         */

        {
            let angle_dis = (previous_est_angle - angle_obs).abs();
            // 首先不论谁减谁得到的结果都是两个角度的差值
            if angle_dis > (PI/2f64) as TypeUnderModel && angle_dis < PI as TypeUnderModel {
                /*
                角度差在 [0, π] 之间时
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
            } else if angle_dis > PI as TypeUnderModel && angle_dis < (PI*3f64/2f64) as TypeUnderModel {  // 在 [π, 3π/2] 之间
                previous_est_angle = Self::wrap_2_minus_pi_2_pi(previous_est_angle+PI as TypeUnderModel);
            } else {  // 在 [3π/2, 2π] 之间
                previous_est_angle = Self::wrap_2_minus_pi_2_pi(previous_est_angle+PI as TypeUnderModel);
                angle_obs = Self::wrap_2_minus_pi_2_pi(angle_obs+PI as TypeUnderModel);
            }
        }

        let mut state_revision = *self.previous_est.state();
        state_revision[6] = previous_est_angle;
        self.previous_est = StateAndCovariance::new(state_revision, *self.previous_est.covariance());
        self.observation = OVector::<TypeUnderModel, U7>::from_iterator([
            x as TypeUnderModel,
            y as TypeUnderModel,
            z as TypeUnderModel,
            h as TypeUnderModel,
            w as TypeUnderModel,
            l as TypeUnderModel,
            angle_obs as TypeUnderModel,
        ]);

        let kf = KalmanFilterNoControl::new(&*MOTION_MODEL, &*OBSERVATION_MODEL);
        // self.previous_est = kf.step(&self.previous_est, &self.observation).unwrap();
        self.previous_est = match kf.step(&self.previous_est, &self.observation) {
            Ok(x) => x,
            Err(e) => {
                println!("{}", e.to_string());
                panic!();
            }
        }
    }
}