#![cfg(feature = "rosrust")]

// Run locally: ./integration/scripts/run-ros-tests-local.sh "rosrust" or in Docker: ./integration/scripts/run-ros-tests-docker.sh rosrust integration/docker/Dockerfile_ros1_noetic

use ros_pointcloud2::PointCloud2Msg;

ros_pointcloud2::impl_pointcloud2_for_rosrust!();

#[test]
fn convertxyz_rosrust_msg() {
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

    let rosrust_msg_cloud: rosrust_msg::sensor_msgs::PointCloud2 =
        crate::impl_rosrust::from_pointcloud2_msg(internal_cloud);
    let convert_back_internal: PointCloud2Msg =
        crate::impl_rosrust::to_pointcloud2_msg(rosrust_msg_cloud);
    let to_convert = convert_back_internal.try_into_iter().unwrap();
    let back_to_type = to_convert.collect::<Vec<PointXYZ>>();
    assert_eq!(copy, back_to_type);
}
