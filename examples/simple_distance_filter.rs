/// This example demonstrates a very simple distance filter with predefined point types.
/// Note that this example is a simplified version of the custom_enum_field_filter.rs example.
/// Also, it effectively demonstrates a typesafe byte-to-byte buffer filter with a single iteration.
///
/// It also works without any dependencies, making it a good "hello world" example.
use ros_pointcloud2::prelude::*;

fn main() {
    let cloud = vec![
        PointXYZ::new(1.0, 1.0, 1.0),
        PointXYZ::new(2.0, 2.0, 2.0),
        PointXYZ::new(3.0, 3.0, 3.0),
    ];

    println!("Original cloud: {:?}", cloud);

    let msg = PointCloud2Msg::try_from_iter(cloud).unwrap();

    println!("filtering by distance < 1.9m");
    let out = msg
        .try_into_iter()
        .unwrap()
        .filter(|point: &PointXYZ| {
            (point.x.powi(2) + point.y.powi(2) + point.z.powi(2)).sqrt() < 1.9
        })
        .collect::<Vec<_>>();

    println!("Filtered cloud: {:?}", out);

    assert_eq!(vec![PointXYZ::new(1.0, 1.0, 1.0),], out);
}
