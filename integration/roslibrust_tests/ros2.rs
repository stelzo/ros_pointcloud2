#![allow(unused_imports)]
use ros_pointcloud2::prelude::*;

#[cfg(feature = "roslibrust")]
mod msg {
    include!(concat!(env!("OUT_DIR"), "/ros2_messages.rs"));
}

#[cfg(feature = "roslibrust")]
impl_pointcloud2_for_roslibrust_ros2!(crate::msg);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(not(feature = "roslibrust"))]
    return Err("roslibrust feature not enabled".into());

    #[cfg(feature = "roslibrust")]
    {
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

        let roslibrust_msg_cloud =
            crate::impl_roslibrust_ros2::from_pointcloud2_msg(internal_cloud);
        let convert_back_internal =
            crate::impl_roslibrust_ros2::to_pointcloud2_msg(roslibrust_msg_cloud);
        let to_convert = convert_back_internal.try_into_iter().unwrap();
        let back_to_type = to_convert.collect::<Vec<PointXYZ>>();
        assert_eq!(copy, back_to_type);

        Ok(())
    }
}
