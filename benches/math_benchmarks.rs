use std::hint::black_box;
use criterion::{criterion_group, criterion_main, Criterion};

fn vec3(c: &mut Criterion) {
    // vec3 add
    let mut group = c.benchmark_group("vec3");
    group.bench_function("nalgebra vec3 add", |bencher| {
        use nalgebra::Vector3;
        let a = Vector3::new(1.0, 2.0, 3.0);
        let b = Vector3::new(4.0, 5.0, 6.0);
        bencher.iter(|| *black_box(&a) + *black_box(&b))
    });
    group.bench_function("wide vec3 add", |bencher| {
        use wide::f64x4;
        let a = f64x4::from([1.0, 2.0, 3.0, 0.0]);
        let b = f64x4::from([4.0, 5.0, 6.0, 0.0]);
        bencher.iter(|| *black_box(&a) + *black_box(&b))
    });
    // vec3 sub
    group.bench_function("nalgebra vec3 sub", |bencher| {
        use nalgebra::Vector3;
        let a = Vector3::new(1.0, 2.0, 3.0);
        let b = Vector3::new(4.0, 5.0, 6.0);
        bencher.iter(|| *black_box(&a) - *black_box(&b))
    });
    group.bench_function("wide vec3 sub", |bencher| {
        use wide::f64x4;
        let a = f64x4::from([1.0, 2.0, 3.0, 0.0]);
        let b = f64x4::from([4.0, 5.0, 6.0, 0.0]);
        bencher.iter(|| *black_box(&a) - *black_box(&b))
    });
    // vec3 dot
    group.bench_function("nalgebra vec3 dot", |bencher| {
        use nalgebra::Vector3;
        let a = Vector3::new(1.0, 2.0, 3.0);
        let b = Vector3::new(4.0, 5.0, 6.0);
        bencher.iter(|| black_box(&a).dot(black_box(&b)))
    });
    group.bench_function("wide vec3 dot", |bencher| {
        use wide::f64x4;
        let a = f64x4::from([1.0, 2.0, 3.0, 0.0]);
        let b = f64x4::from([4.0, 5.0, 6.0, 0.0]);
        bencher.iter(|| {
            let prod = *black_box(&a) * *black_box(&b);
            prod.reduce_add()
        })
    });
    // vec3 cross
    group.bench_function("nalgebra vec3 cross", |bencher| {
        use nalgebra::Vector3;
        let a = Vector3::new(1.0, 2.0, 3.0);
        let b = Vector3::new(4.0, 5.0, 6.0);
        bencher.iter(|| black_box(&a).cross(black_box(&b)))
    });
    group.bench_function("wide vec3 cross", |bencher| {
        use wide::{f64x4, u64x4};
        let a = f64x4::from([1.0, 2.0, 3.0, 0.0]);
        let b = f64x4::from([4.0, 5.0, 6.0, 0.0]);
        bencher.iter(|| {
            let a_arr = black_box(a.as_array_ref());
            let b_arr = black_box(b.as_array_ref());
            // permute vector

            let a_yzx = f64x4::from([a_arr[1], a_arr[2], a_arr[0], 0.0]);
            let b_yzx = f64x4::from([b_arr[1], b_arr[2], b_arr[0], 0.0]);
            let c = a * b_yzx - a_yzx * b;
            let c_arr = c.as_array_ref();
            f64x4::from([c_arr[1], c_arr[2], c_arr[0], 0.0])
        })
    });
    group.finish();
}

criterion_group!(
    benches, vec3,
    // mat3,
    // vec4,
    // mat4,
    // vec6,
    // mat6
);

criterion_main!(benches);
