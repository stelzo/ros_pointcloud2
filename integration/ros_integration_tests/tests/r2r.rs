#![cfg(feature = "r2r")]

// Run with: cargo test -p ros_integration_tests --features r2r

use ros_pointcloud2::PointCloud2Msg;

ros_pointcloud2::impl_pointcloud2_for_r2r!();

#[test]
fn convertxyz_r2r_msg() {
    use ::r2r::sensor_msgs::msg::PointCloud2;
    use ros_pointcloud2::points::PointXYZ;

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
    let internal_cloud = PointCloud2Msg::try_from_iter(&cloud).unwrap();

    let r2r_msg_cloud: PointCloud2 = crate::impl_r2r::from_pointcloud2_msg(internal_cloud);
    let convert_back_internal: PointCloud2Msg = crate::impl_r2r::to_pointcloud2_msg(r2r_msg_cloud);
    let to_convert = convert_back_internal.try_into_iter().unwrap();
    let back_to_type = to_convert.collect::<Vec<PointXYZ>>();
    assert_eq!(copy, back_to_type);
}
