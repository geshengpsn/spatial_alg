use nalgebra::{Matrix3, Matrix4, Vector3, Vector6};

use crate::Real;

pub(crate) fn approx_zero<T: Real>(v: T) -> bool {
    v < T::epsilon()
}

pub(crate) fn approx_zero_vec<T: Real>(v: &Vector3<T>) -> bool {
    length(v) < T::epsilon()
}

pub(crate) fn axis_angle<T: Real>(v: &Vector3<T>) -> (Vector3<T>, T) {
    let angle = length(v);
    (v / angle, angle)
}

pub(crate) fn length<T: Real>(v: &Vector3<T>) -> T {
    (v.x * v.x + v.y * v.y + v.z * v.z).sqrt()
}

/// hat operator for vector3
pub fn hat<T: Real>(v: &Vector3<T>) -> Matrix3<T> {
    let zero = T::zero();
    Matrix3::new(
        zero, -v[2], v[1], 
        v[2], zero, -v[0], 
        -v[1], v[0], zero)
}

pub fn vee<T: Real>(m: &Matrix3<T>) -> Vector3<T> {
    Vector3::new(m[(2, 1)], m[(0, 2)], m[(1, 0)])
}

// /// hat operator for vector6
pub fn hat_se3<T: Real>(v: &Vector6<T>) -> Matrix4<T> {
    let zero = T::zero();
    Matrix4::new(
        zero, -v[2], v[1], v[3], 
        v[2], zero, -v[0], v[4], 
        -v[1], v[0], zero, v[5], 
        zero, zero, zero, zero,
    )
}

pub fn vee_se3<T: Real>(m: &Matrix4<T>) -> Vector6<T> {
    Vector6::new(
        m[(2, 1)], m[(0, 2)], m[(1, 0)],
        m[(0, 3)], m[(1, 3)], m[(2, 3)],
    )
}