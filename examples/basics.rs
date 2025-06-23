use spatial_alg::prelude::*;
use nalgebra::Matrix4;

fn main() {
    let a_t_b = Matrix4::<f64>::identity();
    let b = a_t_b.log();
}