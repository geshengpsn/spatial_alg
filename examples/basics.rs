use spatial_alg::prelude::*;
use nalgebra::Matrix4;

fn main() {
    let a_t_b = Matrix4::<f64>::identity();
    let b = a_t_b.log();
    println!("a_t_b: {a_t_b:.2}");
    println!("b: {b:.2}");
}