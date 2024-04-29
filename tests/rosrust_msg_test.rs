#[cfg(feature = "rosrust_msg")]
#[test]
fn convertxyz_rosrust_msg() {
    use ros_pointcloud2::{
        pcl_utils::PointXYZ, reader::ReaderXYZ, writer::WriterXYZ, PointCloud2Msg,
    };

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
    let internal_cloud: PointCloud2Msg = WriterXYZ::from(cloud).try_into().unwrap();
    let rosrust_msg_cloud: rosrust_msg::sensor_msgs::PointCloud2 = internal_cloud.into();
    let convert_back_internal: PointCloud2Msg = rosrust_msg_cloud.into();
    let to_convert: ConvertXYZ = ReaderXYZ::try_from(convert_back_internal).unwrap();
    let back_to_type = to_convert.map(|point| Ok(point)).collect::<Vec<PointXYZ>>();
    assert_eq!(copy, back_to_type.unwrap());
}
