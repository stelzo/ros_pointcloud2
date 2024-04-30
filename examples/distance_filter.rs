// This example demonstrates a very simple distance filter with predefined point types.
// Note that this example is a simplified version of the custom_filter.rs example.
// Also, it effectively demonstrates a typesafe byte-to-byte buffer filter with a single iteration.

use ros_pointcloud2::{pcl_utils::PointXYZ, PointCloud2Msg};

fn main() {
    let cloud = vec![
        PointXYZ {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },
        PointXYZ {
            x: 2.0,
            y: 2.0,
            z: 2.0,
        },
        PointXYZ {
            x: 3.0,
            y: 3.0,
            z: 3.0,
        },
    ];

    println!("Original cloud: {:?}", cloud);

    let msg = PointCloud2Msg::try_from_iterable(cloud.clone()).unwrap();

    println!("filtering by distance < 1.9m");
    let out = msg
        .try_into_iter()
        .unwrap()
        .filter(|point: &PointXYZ| {
            (point.x.powi(2) + point.y.powi(2) + point.z.powi(2)).sqrt() < 1.9
        })
        .collect::<Vec<_>>();

    println!("Filtered cloud: {:?}", out);

    assert_eq!(
        vec![PointXYZ {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },],
        out
    );
}
