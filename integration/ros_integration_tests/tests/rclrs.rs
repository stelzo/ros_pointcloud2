#![cfg(feature = "rclrs")]

// Run with: cargo test -p ros_integration_tests --features rclrs

use ros_pointcloud2::PointCloud2Msg;

ros_pointcloud2::impl_pointcloud2_for_rclrs!();

#[test]
fn convertxyz_rclrs_msg() {
    use ros_pointcloud2::points::PointXYZ;

    let cloud = vec![
        PointXYZ { x: 1.0, y: 2.0, z: 3.0 },
        PointXYZ { x: 4.0, y: 5.0, z: 6.0 },
        PointXYZ { x: 7.0, y: 8.0, z: 9.0 },
    ];

    let copy = cloud.clone();
    let internal_cloud = PointCloud2Msg::try_from_iter(&cloud).unwrap();

    let rclrs_msg_cloud: ::rclrs::sensor_msgs::msg::PointCloud2 =
        crate::impl_rclrs::from_pointcloud2_msg(internal_cloud);
    let convert_back_internal: PointCloud2Msg =
        crate::impl_rclrs::to_pointcloud2_msg(rclrs_msg_cloud);
    let to_convert = convert_back_internal.try_into_iter().unwrap();
    let back_to_type = to_convert.collect::<Vec<PointXYZ>>();
    assert_eq!(copy, back_to_type);
}
