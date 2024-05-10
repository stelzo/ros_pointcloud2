use criterion::{black_box, criterion_group, criterion_main, Criterion};
use ros_pointcloud2::prelude::*;

use rand::Rng;

pub type PointXYZB = PointXYZINormal;

pub fn distance_to_origin(point: &PointXYZ) -> f32 {
    ((point.x.powi(2)) + (point.y.powi(2)) + (point.z.powi(2))).sqrt()
}

pub fn dot_product(point1: &PointXYZ, point2: &PointXYZ) -> f32 {
    point1.x * point2.x + point1.y * point2.y + point1.z * point2.z
}

pub fn cross_product(point1: &PointXYZ, point2: &PointXYZ) -> PointXYZ {
    PointXYZ {
        x: point1.y * point2.z - point1.z * point2.y,
        y: point1.z * point2.x - point1.x * point2.z,
        z: point1.x * point2.y - point1.y * point2.x,
    }
}

pub fn scalar_multiply(point: &PointXYZ, scalar: f32) -> PointXYZ {
    PointXYZ {
        x: point.x * scalar,
        y: point.y * scalar,
        z: point.z * scalar,
    }
}

pub fn magnitude_squared(point: &PointXYZ) -> f32 {
    (point.x.powi(2)) + (point.y.powi(2)) + (point.z.powi(2))
}

pub fn reflection_through_plane(
    point: &PointXYZ,
    normal: &PointXYZ,
    point_on_plane: &PointXYZ,
) -> PointXYZ {
    PointXYZ {
        x: point.x
            - 2.0
                * ((point.x - point_on_plane.x) * normal.x
                    + (point.y - point_on_plane.y) * normal.y
                    + (point.z - point_on_plane.z) * normal.z),
        y: point.y
            - 2.0
                * ((point.x - point_on_plane.x) * normal.x
                    + (point.y - point_on_plane.y) * normal.y
                    + (point.z - point_on_plane.z) * normal.z),
        z: point.z
            - 2.0
                * ((point.x - point_on_plane.x) * normal.x
                    + (point.y - point_on_plane.y) * normal.y
                    + (point.z - point_on_plane.z) * normal.z),
    }
}

pub fn rotation_about_x(point: &PointXYZ, angle: f32) -> PointXYZ {
    let c = f32::cos(angle);
    let s = f32::sin(angle);
    PointXYZ {
        x: point.x,
        y: point.y * c - point.z * s,
        z: point.y * s + point.z * c,
    }
}

pub fn closest_point_on_line(
    point: &PointXYZ,
    line_point: &PointXYZ,
    line_direction: &PointXYZ,
) -> PointXYZ {
    PointXYZ {
        x: line_point.x
            + (line_point.x - point.x) * ((line_point.x - point.x).powi(2))
                / ((line_direction.x * 2.0).powi(2))
            + (line_direction.y * 2.0) * (point.z - line_point.z)
                / ((line_direction.z * 2.0).powi(2)),
        y: line_point.y
            + (line_point.y - point.y) * ((line_point.y - point.y).powi(2))
                / ((line_direction.y * 2.0).powi(2))
            + (line_direction.x * 2.0) * (point.x - line_point.x)
                / ((line_direction.x * 2.0).powi(2)),
        z: line_point.z
            + (line_point.z - point.z) * ((line_point.z - point.z).powi(2))
                / ((line_direction.z * 2.0).powi(2))
            + (line_direction.y * 2.0) * (point.y - line_point.y)
                / ((line_direction.y * 2.0).powi(2)),
    }
}

fn minus(point1: &PointXYZ, point2: &PointXYZ) -> PointXYZ {
    PointXYZ {
        x: point1.x - point2.x,
        y: point1.y - point2.y,
        z: point1.z - point2.z,
    }
}

pub fn generate_random_pointcloud(num_points: usize, min: f32, max: f32) -> Vec<PointXYZB> {
    let mut rng = rand::thread_rng();
    let mut pointcloud = Vec::with_capacity(num_points);
    for _ in 0..num_points {
        let point = PointXYZB {
            x: rng.gen_range(min..max),
            y: rng.gen_range(min..max),
            z: rng.gen_range(min..max),
            ..Default::default()
        };
        pointcloud.push(point);
    }
    pointcloud
}

pub fn heavy_computing(point: &PointXYZ, iterations: u32) -> f32 {
    let mut result = distance_to_origin(point);
    for _ in 0..iterations {
        result += dot_product(
            point,
            &PointXYZ {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
        );
        result += cross_product(
            point,
            &PointXYZ {
                x: 4.0,
                y: 5.0,
                z: 6.0,
            },
        )
        .x;
        result = result + (result * 10.0).sqrt();
        let reflected_point = reflection_through_plane(
            point,
            &PointXYZ {
                x: 7.0,
                y: 8.0,
                z: 9.0,
            },
            &PointXYZ {
                x: 3.0,
                y: 4.0,
                z: 5.0,
            },
        );
        let rotated_point = rotation_about_x(
            &PointXYZ {
                x: 10.0,
                y: 11.0,
                z: 12.0,
            },
            std::f32::consts::PI / 2.0,
        );

        result += magnitude_squared(&minus(&reflected_point, &rotated_point));
    }
    result
}

#[cfg(feature = "derive")]
fn roundtrip_vec(cloud: Vec<PointXYZB>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_vec(cloud).unwrap();
    let total: Vec<PointXYZ> = internal_msg.try_into_vec().unwrap();
    orig_len == total.len()
}

fn roundtrip(cloud: Vec<PointXYZB>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_iter()
        .unwrap()
        .collect::<Vec<PointXYZ>>();
    orig_len == total.len()
}

#[cfg(feature = "derive")]
fn roundtrip_filter_vec(cloud: Vec<PointXYZB>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_vec(cloud).unwrap();
    let total = internal_msg
        .try_into_iter()
        .unwrap()
        .filter(|point: &PointXYZ| distance_to_origin(point) < 69.9)
        .fold(PointXYZ::default(), |acc, point| PointXYZ {
            x: acc.x + point.x,
            y: acc.y + point.y,
            z: acc.z + point.z,
        });
    orig_len == total.x as usize
}

fn roundtrip_filter(cloud: Vec<PointXYZB>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_iter()
        .unwrap()
        .filter(|point: &PointXYZ| distance_to_origin(point) < 69.9)
        .fold(PointXYZ::default(), |acc, point| PointXYZ {
            x: acc.x + point.x,
            y: acc.y + point.y,
            z: acc.z + point.z,
        });
    orig_len == total.x as usize
}

fn roundtrip_computing(cloud: Vec<PointXYZB>) -> bool {
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_iter()
        .unwrap()
        .map(|point: PointXYZ| heavy_computing(&point, 100))
        .sum::<f32>();
    total > 0.0
}

#[cfg(feature = "rayon")]
fn roundtrip_computing_par(cloud: Vec<PointXYZB>) -> bool {
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_par_iter()
        .unwrap()
        .map(|point: PointXYZ| heavy_computing(&point, 100))
        .sum::<f32>();
    total > 0.0
}

#[cfg(feature = "rayon")]
fn roundtrip_computing_par_par(cloud: Vec<PointXYZB>) -> bool {
    let internal_msg = PointCloud2Msg::try_from_par_iter(cloud.into_par_iter()).unwrap();
    let total = internal_msg
        .try_into_par_iter()
        .unwrap()
        .map(|point: PointXYZ| heavy_computing(&point, 100))
        .sum::<f32>();
    total > 0.0
}

#[cfg(feature = "derive")]
fn roundtrip_computing_vec(cloud: Vec<PointXYZB>) -> bool {
    let internal_msg = PointCloud2Msg::try_from_vec(cloud).unwrap();
    let total: f32 = internal_msg
        .try_into_vec()
        .unwrap()
        .into_iter()
        .map(|point: PointXYZ| heavy_computing(&point, 100))
        .sum();
    total > 0.0
}

#[cfg(feature = "rayon")]
fn roundtrip_par(cloud: Vec<PointXYZB>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_par_iter()
        .unwrap()
        .collect::<Vec<PointXYZ>>();
    orig_len != total.len()
}

#[cfg(feature = "rayon")]
fn roundtrip_par_par(cloud: Vec<PointXYZB>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_par_iter(cloud.into_par_iter()).unwrap();
    let total = internal_msg
        .try_into_par_iter()
        .unwrap()
        .collect::<Vec<PointXYZ>>();
    orig_len != total.len()
}

#[cfg(feature = "rayon")]
fn roundtrip_filter_par(cloud: Vec<PointXYZB>) -> bool {
    let orig_len: usize = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_par_iter()
        .unwrap()
        .filter(|point: &PointXYZ| distance_to_origin(point) < 69.9)
        .reduce(PointXYZ::default, |acc, point| PointXYZ {
            x: acc.x + point.x,
            y: acc.y + point.y,
            z: acc.z + point.z,
        });
    orig_len == total.x as usize
}

#[cfg(feature = "rayon")]
fn roundtrip_filter_par_par(cloud: Vec<PointXYZB>) -> bool {
    let orig_len: usize = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_par_iter(cloud.into_par_iter()).unwrap();
    let total = internal_msg
        .try_into_par_iter()
        .unwrap()
        .filter(|point: &PointXYZ| distance_to_origin(point) < 69.9)
        .reduce(PointXYZ::default, |acc, point| PointXYZ {
            x: acc.x + point.x,
            y: acc.y + point.y,
            z: acc.z + point.z,
        });
    orig_len == total.x as usize
}

fn roundtrip_benchmark(c: &mut Criterion) {
    let cloud_points_16k = generate_random_pointcloud(16_000, f32::MIN / 2.0, f32::MAX / 2.0);
    let cloud_points_60k = generate_random_pointcloud(60_000, f32::MIN / 2.0, f32::MAX / 2.0);
    let cloud_points_120k = generate_random_pointcloud(120_000, f32::MIN / 2.0, f32::MAX / 2.0);
    let cloud_points_500k = generate_random_pointcloud(500_000, f32::MIN / 2.0, f32::MAX / 2.0);
    let cloud_points_1_5m = generate_random_pointcloud(1_500_000, f32::MIN / 2.0, f32::MAX / 2.0);

    // 16k points (Velodyne with 16 beams)

    // Moving memory
    c.bench_function("16k iter", |b| {
        b.iter(|| {
            black_box(roundtrip(cloud_points_16k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("16k iter_par", |b| {
        b.iter(|| {
            black_box(roundtrip_par(cloud_points_16k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("16k iter_par_par", |b| {
        b.iter(|| {
            black_box(roundtrip_par_par(cloud_points_16k.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("16k vec", |b| {
        b.iter(|| {
            black_box(roundtrip_vec(cloud_points_16k.clone()));
        })
    });

    // Simple distance filter
    c.bench_function("16k iter_filter", |b| {
        b.iter(|| {
            roundtrip_filter(black_box(cloud_points_16k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("16k filter_par", |b| {
        b.iter(|| {
            roundtrip_filter_par(black_box(cloud_points_16k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("16k filter_par_par", |b| {
        b.iter(|| {
            black_box(roundtrip_filter_par_par(cloud_points_16k.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("16k vec_filter", |b| {
        b.iter(|| {
            roundtrip_filter_vec(black_box(cloud_points_16k.clone()));
        })
    });

    // Heavy computing
    c.bench_function("16k iter_compute", |b| {
        b.iter(|| {
            roundtrip_computing(black_box(cloud_points_16k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("16k iter_compute_par", |b| {
        b.iter(|| {
            roundtrip_computing_par(black_box(cloud_points_16k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("16k iter_compute_par_par", |b| {
        b.iter(|| {
            roundtrip_computing_par_par(black_box(cloud_points_16k.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("16k vec_compute", |b| {
        b.iter(|| {
            roundtrip_computing_vec(black_box(cloud_points_16k.clone()));
        })
    });

    // 60k points (Ouster with 64 beams)

    // Moving memory
    c.bench_function("60k iter", |b| {
        b.iter(|| {
            black_box(roundtrip(cloud_points_60k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("60k iter_par", |b| {
        b.iter(|| {
            black_box(roundtrip_par(cloud_points_60k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("60k iter_par_par", |b| {
        b.iter(|| {
            black_box(roundtrip_par_par(cloud_points_60k.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("60k vec", |b| {
        b.iter(|| {
            black_box(roundtrip_vec(cloud_points_60k.clone()));
        })
    });

    // 120k points (Ouster with 128 beams)

    // Moving memory
    c.bench_function("120k iter", |b| {
        b.iter(|| {
            black_box(roundtrip(cloud_points_120k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("120k iter_par", |b| {
        b.iter(|| {
            black_box(roundtrip_par(cloud_points_120k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("120k iter_par_par", |b| {
        b.iter(|| {
            black_box(roundtrip_par_par(cloud_points_120k.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("120k vec", |b| {
        b.iter(|| {
            black_box(roundtrip_vec(cloud_points_120k.clone()));
        })
    });

    // Simple distance filter
    c.bench_function("120k iter_filter", |b| {
        b.iter(|| {
            roundtrip_filter(black_box(cloud_points_120k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("120k filter_par", |b| {
        b.iter(|| {
            roundtrip_filter_par(black_box(cloud_points_120k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("120k filter_par_par", |b| {
        b.iter(|| {
            black_box(roundtrip_filter_par_par(cloud_points_120k.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("120k vec_filter", |b| {
        b.iter(|| {
            roundtrip_filter_vec(black_box(cloud_points_120k.clone()));
        })
    });

    // Heavy computing
    c.bench_function("120k iter_compute", |b| {
        b.iter(|| {
            roundtrip_computing(black_box(cloud_points_120k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("120k iter_compute_par", |b| {
        b.iter(|| {
            roundtrip_computing_par(black_box(cloud_points_120k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("120k iter_compute_par_par", |b| {
        b.iter(|| {
            roundtrip_computing_par_par(black_box(cloud_points_120k.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("120k vec_compute", |b| {
        b.iter(|| {
            roundtrip_computing_vec(black_box(cloud_points_120k.clone()));
        })
    });

    // 500k points (just to show how it scales)

    // Moving memory
    c.bench_function("500k iter", |b| {
        b.iter(|| {
            black_box(roundtrip(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("500k iter_par", |b| {
        b.iter(|| {
            black_box(roundtrip_par(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("500k iter_par_par", |b| {
        b.iter(|| {
            black_box(roundtrip_par_par(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("500k vec", |b| {
        b.iter(|| {
            black_box(roundtrip_vec(cloud_points_500k.clone()));
        })
    });

    // Simple distance filter
    c.bench_function("500k iter_filter", |b| {
        b.iter(|| {
            roundtrip_filter(black_box(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("500k filter_par", |b| {
        b.iter(|| {
            roundtrip_filter_par(black_box(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("500k filter_par_par", |b| {
        b.iter(|| {
            black_box(roundtrip_filter_par_par(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("500k vec_filter", |b| {
        b.iter(|| {
            roundtrip_filter_vec(black_box(cloud_points_500k.clone()));
        })
    });

    // Heavy computing
    c.bench_function("500k iter_compute", |b| {
        b.iter(|| {
            roundtrip_computing(black_box(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("500k iter_compute_par", |b| {
        b.iter(|| {
            roundtrip_computing_par(black_box(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("500k iter_compute_par_par", |b| {
        b.iter(|| {
            roundtrip_computing_par_par(black_box(cloud_points_500k.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("500k vec_compute", |b| {
        b.iter(|| {
            roundtrip_computing_vec(black_box(cloud_points_500k.clone()));
        })
    });

    // 1.5m points (scale of small localmaps in SLAM)

    // Moving memory
    c.bench_function("1.5m iter", |b| {
        b.iter(|| {
            black_box(roundtrip(cloud_points_1_5m.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("1.5m iter_par", |b| {
        b.iter(|| {
            black_box(roundtrip_par(cloud_points_1_5m.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("1.5m iter_par_par", |b| {
        b.iter(|| {
            black_box(roundtrip_par_par(cloud_points_1_5m.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("1.5m vec", |b| {
        b.iter(|| {
            black_box(roundtrip_vec(cloud_points_1_5m.clone()));
        })
    });

    // Simple distance filter
    c.bench_function("1.5m iter_filter", |b| {
        b.iter(|| {
            roundtrip_filter(black_box(cloud_points_1_5m.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("1.5m iter_par_filter", |b| {
        b.iter(|| {
            black_box(roundtrip_filter_par(cloud_points_1_5m.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("1.5m iter_par_par_filter", |b| {
        b.iter(|| {
            black_box(roundtrip_filter_par_par(cloud_points_1_5m.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("1.5m vec_filter", |b| {
        b.iter(|| {
            roundtrip_filter_vec(black_box(cloud_points_1_5m.clone()));
        })
    });

    // Heavy computing
    c.bench_function("1.5m iter_compute", |b| {
        b.iter(|| {
            roundtrip_computing(black_box(cloud_points_1_5m.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("1.5m iter_compute_par", |b| {
        b.iter(|| {
            roundtrip_computing_par(black_box(cloud_points_1_5m.clone()));
        })
    });

    #[cfg(feature = "rayon")]
    c.bench_function("1.5m iter_compute_par_par", |b| {
        b.iter(|| {
            roundtrip_computing_par_par(black_box(cloud_points_1_5m.clone()));
        })
    });

    #[cfg(feature = "derive")]
    c.bench_function("1.5m vec_compute", |b| {
        b.iter(|| {
            roundtrip_computing_vec(black_box(cloud_points_1_5m.clone()));
        })
    });
}

criterion_group!(benches, roundtrip_benchmark);
criterion_main!(benches);
