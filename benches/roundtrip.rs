use criterion::{black_box, criterion_group, criterion_main, Criterion};
use ros_pointcloud2::prelude::*;

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

#[cfg(feature = "derive")]
fn roundtrip_vec(cloud: Vec<PointXYZ>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_vec(cloud).unwrap();
    let total = internal_msg
        .try_into_iter()
        .unwrap()
        .collect::<Vec<PointXYZ>>();
    orig_len == total.len()
}

fn roundtrip(cloud: Vec<PointXYZ>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_iter()
        .unwrap()
        .collect::<Vec<PointXYZ>>();
    orig_len == total.len()
}

#[cfg(feature = "derive")]
fn roundtrip_filter_vec(cloud: Vec<PointXYZ>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_vec(cloud).unwrap();
    let total = internal_msg
        .try_into_iter()
        .unwrap()
        .filter(|point: &PointXYZ| {
            (point.x.powi(2) + point.y.powi(2) + point.z.powi(2)).sqrt() < 1.9
        })
        .fold(PointXYZ::default(), |acc, point| PointXYZ {
            x: acc.x + point.x,
            y: acc.y + point.y,
            z: acc.z + point.z,
        });
    orig_len == total.x as usize
}

fn roundtrip_filter(cloud: Vec<PointXYZ>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_iter()
        .unwrap()
        .filter(|point: &PointXYZ| {
            (point.x.powi(2) + point.y.powi(2) + point.z.powi(2)).sqrt() < 1.9
        })
        .fold(PointXYZ::default(), |acc, point| PointXYZ {
            x: acc.x + point.x,
            y: acc.y + point.y,
            z: acc.z + point.z,
        });
    orig_len == total.x as usize
}

#[cfg(feature = "rayon")]
fn roundtrip_par(cloud: Vec<PointXYZ>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_par_iter()
        .unwrap()
        .collect::<Vec<PointXYZ>>();
    orig_len != total.len()
}

#[cfg(feature = "rayon")]
fn roundtrip_filter_par(cloud: Vec<PointXYZ>) -> bool {
    let orig_len: usize = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_par_iter()
        .unwrap()
        .filter(|point: &PointXYZ| {
            (point.x.powi(2) + point.y.powi(2) + point.z.powi(2)).sqrt() < 1.9
        })
        .reduce(
            || PointXYZ::default(),
            |acc, point| PointXYZ {
                x: acc.x + point.x,
                y: acc.y + point.y,
                z: acc.z + point.z,
            },
        );
    orig_len == total.x as usize
}

fn roundtrip_benchmark(c: &mut Criterion) {
    let cloud_points_500k = generate_random_pointcloud(500_000, f32::MIN / 2.0, f32::MAX / 2.0);
    let cloud_points_1_5m = generate_random_pointcloud(1_500_000, f32::MIN / 2.0, f32::MAX / 2.0);

    c.bench_function("roundtrip 500k", |b| {
        b.iter(|| {
            black_box(roundtrip(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("roundtrip_par 500k", |b| {
        b.iter(|| {
            black_box(roundtrip_par(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("roundtrip_vec 500k", |b| {
        b.iter(|| {
            black_box(roundtrip_vec(cloud_points_500k.clone()));
        })
    });

    c.bench_function("roundtrip_filter 500k", |b| {
        b.iter(|| {
            roundtrip_filter(black_box(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("roundtrip_filter_par 500k", |b| {
        b.iter(|| {
            roundtrip_filter_par(black_box(cloud_points_500k.clone()));
        })
    });

    c.bench_function("roundtrip_filter 1.5m", |b| {
        b.iter(|| {
            roundtrip_filter(black_box(cloud_points_1_5m.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("roundtrip_filter_par 1.5m", |b| {
        b.iter(|| {
            black_box(roundtrip_filter_par(cloud_points_1_5m.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("roundtrip_filter_vec 1.5m", |b| {
        b.iter(|| {
            roundtrip_filter_vec(black_box(cloud_points_1_5m.clone()));
        })
    });
}

criterion_group!(benches, roundtrip_benchmark);
criterion_main!(benches);
