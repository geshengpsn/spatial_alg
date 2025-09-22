use crate::{SO3, se3, utils::approx_zero};
use nalgebra::{Matrix3, Matrix4, Matrix6, Vector3, Vector6};

use crate::{
    Real, SE3,
    utils::{approx_zero_vec, axis_angle, hat},
};

impl<T> SE3 for Matrix4<T>
where
    T: Real,
{
    type se3 = Vector6<T>;

    /// # spatial_alg SE3
    /// log projection
    fn log(&self) -> Vector6<T> {
        let r = self.fixed_view::<3, 3>(0, 0);
        let p = self.fixed_view::<3, 1>(0, 3);
        let w = r.into_owned().log();

        if approx_zero_vec(&w) {
            let mut res = Vector6::zeros();
            res.view_mut((3, 0), (3, 1)).copy_from(&p);
            res
        } else {
            let two = T::one() + T::one();
            let (w_, theta) = axis_angle(&w);
            let w_so3 = hat(&w_);
            let v = {
                let a = Matrix3::identity() / theta;
                let b = w_so3 / two;
                let c = T::one() / theta - T::one() / (theta / two).tan() / two; // * w_so3 * w_so3;
                let c = w_so3 * w_so3 * c;
                (a - b + c) * p
            };
            let mut res = Vector6::zeros();
            res.view_mut((0, 0), (3, 1)).copy_from(&w);
            res.view_mut((3, 0), (3, 1)).copy_from(&(v * theta));
            res
        }
    }

    /// # spatial_alg SE3
    /// inv
    fn inv(&self) -> Self {
        let r = self.fixed_view::<3, 3>(0, 0);
        let p = self.fixed_view::<3, 1>(0, 3);

        let r_inv = r.transpose();
        let p_new = -&r_inv * p;
        let mut res = Matrix4::identity();
        res.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_inv);
        res.fixed_view_mut::<3, 1>(0, 3).copy_from(&p_new);
        res
    }

    type adjoint = Matrix6<T>;
    /// # spatial_alg SE3
    /// adjoint of lee group
    fn lee_adjoint(&self) -> Self::adjoint {
        let r = self.fixed_view::<3, 3>(0, 0);
        let p = self.fixed_view::<3, 1>(0, 3);

        let p_so3 = hat(&p.into());
        let mut res = Matrix6::zeros();
        res.view_mut((0, 0), (3, 3)).copy_from(&r);
        res.view_mut((3, 0), (3, 3)).copy_from(&(p_so3 * r));
        res.view_mut((3, 3), (3, 3)).copy_from(&r);
        res
    }
}

impl<T> se3 for Vector6<T>
where
    T: Real,
{
    type SE3 = Matrix4<T>;

    fn exp(&self) -> Self::SE3 {
        let w = self.fixed_view::<3, 1>(0, 0).into();
        let v = self.fixed_view::<3, 1>(3, 0);
        let (axis, theta) = axis_angle(&w);
        if approx_zero(theta) {
            let mut res = Matrix4::identity();
            res.view_mut((0, 3), (3, 1)).copy_from(&v);
            res
        } else {
            let mut res = Matrix4::identity();
            let w_so3 = hat(&axis);
            let w_so3_sq = w_so3 * w_so3;
            let theta_sin = theta.sin();
            let theta_cos = theta.cos();
            let vv = Matrix3::identity() * theta
                + w_so3 * (T::one() - theta_cos)
                + w_so3_sq * (theta - theta_sin);
            let rot = Matrix3::identity() + w_so3 * theta_sin + w_so3_sq * (T::one() - theta_cos);

            res.view_mut((0, 0), (3, 3)).copy_from(&rot);
            res.view_mut((0, 3), (3, 1)).copy_from(&(vv * v / theta));
            res
        }
    }

    type Adj = Matrix6<T>;

    fn adj(&self) -> Self::Adj {
        let w = Vector3::new(self[0], self[1], self[2]);
        let v = Vector3::new(self[3], self[4], self[5]);
        let v = hat(&v);
        let w = hat(&w);

        let mut res = Matrix6::zeros();

        res.view_mut((0, 0), (3, 3)).copy_from(&w);
        res.view_mut((3, 3), (3, 3)).copy_from(&w);
        res.view_mut((3, 0), (3, 3)).copy_from(&v);

        res
    }
}
