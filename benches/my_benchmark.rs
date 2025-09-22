use criterion::{Criterion, criterion_group, criterion_main};
use nalgebra::{Matrix3, Matrix4};
use spatial_alg::algo::{exp_so3_sq};
use std::hint::black_box;

// fn fibonacci(n: u64) -> u64 {
//     match n {
//         0 => 1,
//         1 => 1,
//         n => fibonacci(n - 1) + fibonacci(n - 2),
//     }
// }

// fn criterion_benchmark(c: &mut Criterion) {
//     c.bench_function("fib 20", |b| b.iter(|| fibonacci(black_box(20))));
//     c.bench_function("fib 10", |b| b.iter(|| fibonacci(black_box(10))));
// }

fn exp_so3_bench(c: &mut Criterion) {
    let w_so3 = Matrix3::new(0.0, -0.3, 0.2, 0.3, 0.0, -0.1, -0.2, 0.1, 0.0);
    let w_so3_sq = w_so3 * w_so3;
    let angle = (0.1f64 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3).sqrt();

    c.bench_function("new mat3", |b| {
        b.iter(Matrix3::<f64>::identity)
    });

    c.bench_function("exp_so3_sq", |b| {
        b.iter(|| unsafe { exp_so3_sq(black_box(&w_so3), black_box(&w_so3_sq), black_box(angle)) })
    });
}

fn mat3_multiply_bench(c: &mut Criterion) {
    let a = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
    let b = Matrix3::new(9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0);
    c.bench_function("mat3 multiply", |bencher| {
        bencher.iter(|| black_box(&a) * black_box(&b))
    });
}

fn mat4_multiply_bench(c: &mut Criterion) {
    let a = Matrix4::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0);
    let b = Matrix4::new(16.0, 15.0, 14.0, 13.0, 12.0, 11.0, 10.0, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0);
    c.bench_function("mat4 multiply", |bencher| {
        bencher.iter(|| black_box(&a) * black_box(&b))
    });
}

fn quaternion_multiply_bench(c: &mut Criterion) {
    use nalgebra::Quaternion;
    let a = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    let b = Quaternion::new(4.0, 3.0, 2.0, 1.0);
    c.bench_function("quaternion multiply", |bencher| {
        bencher.iter(|| black_box(&a) * black_box(&b))
    });
}

criterion_group!(benches, exp_so3_bench, mat3_multiply_bench, mat4_multiply_bench, quaternion_multiply_bench);
criterion_main!(benches);
