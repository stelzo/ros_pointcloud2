use criterion::{black_box, criterion_group, criterion_main, Criterion};
use ros_pointcloud2::{pcl_utils::PointXYZ, PointCloud2Msg};

use rand::Rng;

pub fn generate_random_pointcloud(num_points: usize, min: f32, max: f32) -> Vec<PointXYZ> {
    let mut rng = rand::thread_rng();
    let mut pointcloud = Vec::with_capacity(num_points);
    for _ in 0..num_points {
        let point = PointXYZ {
            x: rng.gen_range(min..max),
            y: rng.gen_range(min..max),
            z: rng.gen_range(min..max),
        };
        pointcloud.push(point);
    }
    pointcloud
}

fn roundtrip(cloud: Vec<PointXYZ>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_iterable(cloud).unwrap();
    let total = internal_msg
        .try_into_iter()
        .unwrap()
        .collect::<Vec<PointXYZ>>();
    orig_len == total.len()
}

fn roundtrip_benchmark(c: &mut Criterion) {
    let cloud_points_10k = generate_random_pointcloud(10_000, f32::MIN / 2.0, f32::MAX / 2.0);
    let cloud_points_100k = generate_random_pointcloud(100_000, f32::MIN / 2.0, f32::MAX / 2.0);
    let cloud_points_1m = generate_random_pointcloud(1_000_000, f32::MIN / 2.0, f32::MAX / 2.0);

    c.bench_function("roundtrip 10k", |b| {
        b.iter(|| {
            black_box(roundtrip(cloud_points_10k.clone()));
        })
    });

    c.bench_function("roundtrip 100k", |b| {
        b.iter(|| {
            black_box(roundtrip(cloud_points_100k.clone()));
        })
    });

    c.bench_function("roundtrip 1m", |b| {
        b.iter(|| {
            black_box(roundtrip(cloud_points_1m.clone()));
        })
    });
}

criterion_group!(benches, roundtrip_benchmark);
criterion_main!(benches);
