use nalgebra::{Matrix3, Vector3};

use crate::{
    so3, utils::{approx_zero, axis_angle, hat, length, vee}, Real, SO3
};

impl<T> SO3 for Matrix3<T>
where
    T: Real,
{
    type so3 = Vector3<T>;

    fn log(&self) -> Self::so3 {
        let rot = self;
        let one: T = T::one();
        let two = one + one;
        let cos = (rot.trace() - one) / two;
        if cos >= one {
            Vector3::zeros()
        } else if cos <= -one {
            let res;
            if approx_zero(one + rot[(2, 2)]) {
                res = Vector3::from_column_slice(&[rot[(0, 2)], rot[(1, 2)], one + rot[(2, 2)]])
                    / (two * (one + rot[(2, 2)])).sqrt();
            } else if approx_zero(one + rot[(1, 1)]) {
                res = Vector3::from_column_slice(&[rot[(0, 1)], one + rot[(1, 1)], rot[(2, 1)]])
                    / (two * (one + rot[(1, 1)])).sqrt();
            } else {
                res = Vector3::from_column_slice(&[rot[(0, 0)], rot[(1, 0)], one + rot[(2, 0)]])
                    / (two * (one + rot[(0, 0)])).sqrt();
            }
            res * T::PI()
        } else {
            let theta = cos.acos();
            let a = (rot - rot.transpose()) * (theta / two / theta.sin());
            Vector3::new(a[(2, 1)], a[(0, 2)], a[(1, 0)])
        }
    }

    fn inv(&self) -> Self {
        self.transpose()
    }

    // fn adjoint(&self) -> Self {
    //     *self
    // }
}

impl<T> so3 for Vector3<T>
where
    T: Real,
{
    type SO3 = Matrix3<T>;

    fn exp(&self) -> Self::SO3 {
        if approx_zero(length(self)) {
            Matrix3::identity()
        } else {
            let (w, angle) = axis_angle(self);
            let w_so3 = hat(&w);
            Matrix3::identity() + w_so3 * angle.sin() + w_so3 * w_so3 * (T::one() - angle.cos())
        }
    }
}

// impl<T> so3 for Matrix3<T>
// where
//     T: Real,
// {
//     type SO3 = Matrix3<T>;

//     fn exp(&self) -> Self::SO3 {
//         vee(self).exp()
//     }
// }