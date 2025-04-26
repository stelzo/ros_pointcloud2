use rand::Rng;
/// This example implements a naive benchmark for the library so you can evaluate the use of rayon for parallel processing.
/// It generates a random point cloud and measures the time it takes to iterate over it.
/// The code works mainly as a showcase. For actual benchmarks, check the `benches` directory or run `cargo bench`.
use std::time::Duration;

use ros_pointcloud2::prelude::*;

pub fn generate_random_pointcloud(num_points: usize, min: f32, max: f32) -> Vec<PointXYZ> {
    let mut rng = rand::rng();
    let mut pointcloud = Vec::with_capacity(num_points);
    for _ in 0..num_points {
        let point = PointXYZ {
            x: rng.random_range(min..max),
            y: rng.random_range(min..max),
            z: rng.random_range(min..max),
        };
        pointcloud.push(point);
    }
    pointcloud
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
        .reduce(PointXYZ::default, |acc, point| PointXYZ {
            x: acc.x + point.x,
            y: acc.y + point.y,
            z: acc.z + point.z,
        });
    orig_len == total.x as usize
}

// call measure_func X times and print the average time
fn measure_func_avg(
    num_iterations: u32,
    pcl_size: usize,
    func: fn(Vec<PointXYZ>) -> bool,
) -> Duration {
    let mut total_time = Duration::new(0, 0);
    for _ in 0..num_iterations {
        total_time += measure_func(pcl_size, func);
    }
    total_time / num_iterations
}

fn measure_func<F>(pcl_size: usize, func: F) -> Duration
where
    F: Fn(Vec<PointXYZ>) -> bool,
{
    let cloud_points = generate_random_pointcloud(pcl_size, f32::MIN / 2.0, f32::MAX / 2.0);
    let start = std::time::Instant::now();
    let _ = func(cloud_points);
    start.elapsed()
}

fn main() {
    println!("100k");
    let how_many = 10_000;
    let how_often = 1_000;

    let dur = measure_func_avg(how_often, how_many, roundtrip);
    println!("roundtrip: {dur:?}");

    #[cfg(feature = "rayon")]
    let dur = measure_func_avg(how_often, how_many, roundtrip_par);
    println!("roundtrip_par: {dur:?}");

    println!("200k");
    let how_many = 200_000;
    let how_often = 100;

    let dur = measure_func_avg(how_often, how_many, roundtrip_filter);
    println!("roundtrip_filter: {dur:?}");

    #[cfg(feature = "rayon")]
    let dur = measure_func_avg(how_often, how_many, roundtrip_filter_par);
    println!("roundtrip_filter_par: {dur:?}");

    println!("10m");
    let how_many = 10_000_000;
    let how_often = 10;

    let dur = measure_func_avg(how_often, how_many, roundtrip_filter);
    println!("roundtrip_filter: {dur:?}");

    #[cfg(feature = "rayon")]
    let dur = measure_func_avg(how_often, how_many, roundtrip_filter_par);
    println!("roundtrip_filter_par: {dur:?}");
}
