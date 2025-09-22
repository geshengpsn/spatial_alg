// so3

use nalgebra::{Matrix3, Vector3};

use crate::{utils::{approx_zero, axis_angle, hat, length}, Real};

pub fn exp_so3<T: Real>(t: &Vector3<T>) -> Matrix3<T> {
    if approx_zero(length(t)) {
        Matrix3::identity()
    } else {
        let (w, angle) = axis_angle(t);
        let w_so3 = hat(&w);
        unsafe { exp_so3_sq(&w_so3, &(w_so3 * w_so3), angle) }
    }
}

/// so3 with precomputed w_so3 and w_so3^2
/// 
/// # Safety
/// The caller must ensure that:
/// - `w` is a unit vector,
/// - `w_so3` corresponds to the skew-symmetric matrix of `w`,
/// - `w_so3_sq` is the square of `w_so3`,
/// - `angle` is the angle of rotation in radians,
/// - The mathematical invariants described above are upheld.
#[inline]
pub unsafe fn exp_so3_sq<T: Real>(w_so3: &Matrix3<T>, w_so3_sq: &Matrix3<T>, angle: T) -> Matrix3<T> {
    let mut res = Matrix3::identity();
    res += w_so3 * angle.sin();
    res += w_so3_sq * (T::one() - angle.cos());
    res
}