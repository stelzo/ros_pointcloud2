#[cfg(feature = "ros2-interfaces-jazzy")]
#[test]
fn convertxyz_ros2_interfaces_jazzy_msg() {
    use ros_pointcloud2::{points::PointXYZ, PointCloud2Msg};

    use ros2_interfaces_jazzy::sensor_msgs::msg::PointCloud2;

    let cloud = vec![
        PointXYZ {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
        PointXYZ {
            x: 4.0,
            y: 5.0,
            z: 6.0,
        },
        PointXYZ {
            x: 7.0,
            y: 8.0,
            z: 9.0,
        },
    ];
    let copy = cloud.clone();
    let internal_cloud = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let ros2_msg_cloud: PointCloud2 = internal_cloud.into();
    let convert_back_internal: PointCloud2Msg = ros2_msg_cloud.into();
    let to_convert = convert_back_internal.try_into_iter().unwrap();
    let back_to_type = to_convert.collect::<Vec<PointXYZ>>();
    assert_eq!(copy, back_to_type);
}
