use ros_pointcloud2::pcl_utils::*;
use ros_pointcloud2::PointCloud2Msg;
use ros_pointcloud2::*;
use std::fmt::Debug;

macro_rules! convert_from_into {
    ($point:ty, $cloud:expr) => {
        convert_from_into_in_out_cloud!($cloud, $point, $cloud, $point);
    };
}

#[cfg(feature = "derive")]
macro_rules! convert_from_into_vec {
    ($point:ty, $cloud:expr) => {
        convert_from_into_in_out_cloud_vec!($cloud, $point, $cloud, $point);
    };
}

macro_rules! convert_from_into_in_out_cloud {
    ($in_cloud:expr, $in_point:ty, $out_cloud:expr, $out_point:ty) => {
        let msg = PointCloud2Msg::try_from_iter($in_cloud.clone().into_iter());
        assert!(msg.is_ok());
        let msg = msg.unwrap();
        let to_p_type = msg.try_into_iter();
        assert!(to_p_type.is_ok());
        let to_p_type = to_p_type.unwrap();
        let back_to_type = to_p_type.collect::<Vec<$out_point>>();
        let orig_cloud: Vec<$out_point> = $out_cloud.iter().cloned().collect();
        assert_eq!(orig_cloud, back_to_type);
    };
}

#[cfg(feature = "derive")]
macro_rules! convert_from_into_in_out_cloud_vec {
    ($in_cloud:expr, $in_point:ty, $out_cloud:expr, $out_point:ty) => {
        let msg = PointCloud2Msg::try_from_vec($in_cloud.clone());
        assert!(msg.is_ok());
        let msg = msg.unwrap();
        let to_p_type = msg.try_into_iter();
        assert!(to_p_type.is_ok());
        let to_p_type = to_p_type.unwrap();
        let back_to_type = to_p_type.collect::<Vec<$out_point>>();
        let orig_cloud: Vec<$out_point> = $out_cloud.iter().cloned().collect();
        assert_eq!(orig_cloud, back_to_type);
    };
}

#[test]
fn write_cloud() {
    let cloud = vec![
        PointXYZ {
            x: 0.0,
            y: 1.0,
            z: 5.0,
        },
        PointXYZ {
            x: 1.0,
            y: 1.5,
            z: 5.0,
        },
        PointXYZ {
            x: 1.3,
            y: 1.6,
            z: 5.7,
        },
        PointXYZ {
            x: f32::MAX,
            y: f32::MIN,
            z: f32::MAX,
        },
    ];

    let msg = PointCloud2Msg::try_from_iter(cloud.into_iter());
    assert!(msg.is_ok());
}

#[test]
#[cfg(feature = "derive")]
fn write_cloud_from_vec() {
    let cloud = vec![
        PointXYZ {
            x: 0.0,
            y: 1.0,
            z: 5.0,
        },
        PointXYZ {
            x: 1.0,
            y: 1.5,
            z: 5.0,
        },
        PointXYZ {
            x: 1.3,
            y: 1.6,
            z: 5.7,
        },
        PointXYZ {
            x: f32::MAX,
            y: f32::MIN,
            z: f32::MAX,
        },
    ];

    let msg = PointCloud2Msg::try_from_vec(cloud);
    assert!(msg.is_ok());

    let msg = msg.unwrap();
    println!("{:?}", msg);
}

#[test]
#[cfg(feature = "derive")]
fn custom_xyz_f32() {
    #[derive(Debug, PartialEq, Clone, Default, RosFull, TypeLayout)]
    #[repr(C)]
    struct CustomPoint {
        x: f32,
        y: f32,
        z: f32,
    }

    convert_from_into!(
        CustomPoint,
        vec![
            CustomPoint {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            CustomPoint {
                x: 4.0,
                y: 5.0,
                z: 6.0,
            },
            CustomPoint {
                x: 7.0,
                y: 8.0,
                z: 9.0,
            },
        ]
    );
}

#[test]
#[cfg(feature = "derive")]
fn custom_xyzi_f32() {
    let cloud: Vec<CustomPointXYZI> = vec![
        CustomPointXYZI {
            x: 0.0,
            y: 1.0,
            z: 5.0,
            i: 0,
        },
        CustomPointXYZI {
            x: 1.0,
            y: 1.5,
            z: 5.0,
            i: 1,
        },
        CustomPointXYZI {
            x: 1.3,
            y: 1.6,
            z: 5.7,
            i: 2,
        },
        CustomPointXYZI {
            x: f32::MAX,
            y: f32::MIN,
            z: f32::MAX,
            i: u8::MAX,
        },
    ];

    #[derive(Debug, PartialEq, Clone, Default, RosFull, TypeLayout)]
    #[repr(C)]
    struct CustomPointXYZI {
        x: f32,
        y: f32,
        z: f32,
        i: u8,
    }

    convert_from_into!(CustomPointXYZI, cloud);
}

#[test]
#[cfg(feature = "derive")]
fn custom_rgba_f32() {
    #[derive(Debug, PartialEq, Clone, Default, RosFull, TypeLayout)]
    #[repr(C)]
    struct CustomPoint {
        x: f32,
        y: f32,
        z: f32,
        r: u8,
        g: u8,
        b: u8,
        a: u8,
    }

    let cloud = vec![
        CustomPoint {
            x: 0.0,
            y: 1.0,
            z: 5.0,
            r: 0,
            g: 0,
            b: 0,
            a: 0,
        },
        CustomPoint {
            x: 1.0,
            y: 1.5,
            z: 5.0,
            r: 1,
            g: 1,
            b: 1,
            a: 1,
        },
        CustomPoint {
            x: 1.3,
            y: 1.6,
            z: 5.7,
            r: 2,
            g: 2,
            b: 2,
            a: 2,
        },
        CustomPoint {
            x: f32::MAX,
            y: f32::MIN,
            z: f32::MAX,
            r: u8::MAX,
            g: u8::MAX,
            b: u8::MAX,
            a: u8::MAX,
        },
    ];
    convert_from_into!(CustomPoint, cloud);
}

#[test]
fn converterxyz() {
    let cloud = vec![
        PointXYZ {
            x: 0.0,
            y: 1.0,
            z: 5.0,
        },
        PointXYZ {
            x: 1.0,
            y: 1.5,
            z: 5.0,
        },
        PointXYZ {
            x: 1.3,
            y: 1.6,
            z: 5.7,
        },
        PointXYZ {
            x: f32::MAX,
            y: f32::MIN,
            z: f32::MAX,
        },
    ];

    convert_from_into!(PointXYZ, cloud);
}

#[test]
fn converterxyzrgba() {
    convert_from_into!(
        PointXYZRGBA,
        vec![
            PointXYZRGBA {
                x: 0.0,
                y: 1.0,
                z: 5.0,
                r: 0,
                g: 0,
                b: 0,
                a: 0,
            },
            PointXYZRGBA {
                x: 1.0,
                y: 1.5,
                z: 5.0,
                r: 1,
                g: 1,
                b: 1,
                a: 1,
            },
            PointXYZRGBA {
                x: 1.3,
                y: 1.6,
                z: 5.7,
                r: 2,
                g: 2,
                b: 2,
                a: 2,
            },
            PointXYZRGBA {
                x: f32::MAX,
                y: f32::MIN,
                z: f32::MAX,
                r: u8::MAX,
                g: u8::MAX,
                b: u8::MAX,
                a: u8::MAX,
            },
        ]
    );
}

#[test]
fn converterxyzinormal() {
    convert_from_into!(
        PointXYZINormal,
        vec![
            PointXYZINormal {
                x: 0.0,
                y: 1.0,
                z: 5.0,
                intensity: 0.0,
                normal_x: 0.0,
                normal_y: 0.0,
                normal_z: 0.0,
            },
            PointXYZINormal {
                x: 1.0,
                y: 1.5,
                z: 5.0,
                intensity: 1.0,
                normal_x: 1.0,
                normal_y: 1.0,
                normal_z: 1.0,
            },
            PointXYZINormal {
                x: 1.3,
                y: 1.6,
                z: 5.7,
                intensity: 2.0,
                normal_x: 2.0,
                normal_y: 2.0,
                normal_z: 2.0,
            },
            PointXYZINormal {
                x: f32::MAX,
                y: f32::MIN,
                z: f32::MAX,
                intensity: f32::MAX,
                normal_x: f32::MAX,
                normal_y: f32::MAX,
                normal_z: f32::MAX,
            },
        ]
    );
}

#[test]
fn converterxyzrgbnormal() {
    convert_from_into!(
        PointXYZRGBNormal,
        vec![
            PointXYZRGBNormal {
                x: 0.0,
                y: 1.0,
                z: 5.0,
                r: 0,
                g: 0,
                b: 0,
                normal_x: 0.0,
                normal_y: 0.0,
                normal_z: 0.0,
            },
            PointXYZRGBNormal {
                x: 1.0,
                y: 1.5,
                z: 5.0,
                r: 1,
                g: 1,
                b: 1,
                normal_x: 1.0,
                normal_y: 1.0,
                normal_z: 1.0,
            },
            PointXYZRGBNormal {
                x: 1.3,
                y: 1.6,
                z: 5.7,
                r: 2,
                g: 2,
                b: 2,
                normal_x: 2.0,
                normal_y: 2.0,
                normal_z: 2.0,
            },
            PointXYZRGBNormal {
                x: f32::MAX,
                y: f32::MIN,
                z: f32::MAX,
                r: u8::MAX,
                g: u8::MAX,
                b: u8::MAX,
                normal_x: f32::MAX,
                normal_y: f32::MAX,
                normal_z: f32::MAX,
            },
        ]
    );
}

#[test]
fn converterxyznormal() {
    convert_from_into!(
        PointXYZNormal,
        vec![
            PointXYZNormal {
                x: 0.0,
                y: 1.0,
                z: 5.0,
                normal_x: 0.0,
                normal_y: 0.0,
                normal_z: 0.0,
            },
            PointXYZNormal {
                x: 1.0,
                y: 1.5,
                z: 5.0,
                normal_x: 1.0,
                normal_y: 1.0,
                normal_z: 1.0,
            },
            PointXYZNormal {
                x: 1.3,
                y: 1.6,
                z: 5.7,
                normal_x: 2.0,
                normal_y: 2.0,
                normal_z: 2.0,
            },
            PointXYZNormal {
                x: f32::MAX,
                y: f32::MIN,
                z: f32::MAX,
                normal_x: f32::MAX,
                normal_y: f32::MAX,
                normal_z: f32::MAX,
            },
        ]
    );
}

#[test]
fn converterxyzrgbl() {
    convert_from_into!(
        PointXYZRGBL,
        vec![
            PointXYZRGBL {
                x: 0.0,
                y: 1.0,
                z: 5.0,
                r: 0,
                g: 0,
                b: 0,
                label: 0,
            },
            PointXYZRGBL {
                x: 1.0,
                y: 1.5,
                z: 5.0,
                r: 1,
                g: 1,
                b: 1,
                label: 1,
            },
            PointXYZRGBL {
                x: 1.3,
                y: 1.6,
                z: 5.7,
                r: 2,
                g: 2,
                b: 2,
                label: 2,
            },
            PointXYZRGBL {
                x: f32::MAX,
                y: f32::MIN,
                z: f32::MAX,
                r: u8::MAX,
                g: u8::MAX,
                b: u8::MAX,
                label: u32::MAX,
            },
        ]
    );
}

#[test]
fn converterxyzrgb() {
    convert_from_into!(
        PointXYZRGB,
        vec![
            PointXYZRGB {
                x: 0.0,
                y: 1.0,
                z: 5.0,
                r: 0,
                g: 0,
                b: 0,
            },
            PointXYZRGB {
                x: 1.0,
                y: 1.5,
                z: 5.0,
                r: 1,
                g: 1,
                b: 1,
            },
            PointXYZRGB {
                x: 1.3,
                y: 1.6,
                z: 5.7,
                r: 2,
                g: 2,
                b: 2,
            },
            PointXYZRGB {
                x: f32::MAX,
                y: f32::MIN,
                z: f32::MAX,
                r: u8::MAX,
                g: u8::MAX,
                b: u8::MAX,
            },
        ]
    );
}

#[test]
#[cfg(feature = "derive")]
fn converterxyzrgb_from_vec() {
    convert_from_into_vec!(
        PointXYZRGB,
        vec![
            PointXYZRGB {
                x: 0.0,
                y: 1.0,
                z: 5.0,
                r: 0,
                g: 0,
                b: 0,
            },
            PointXYZRGB {
                x: 1.0,
                y: 1.5,
                z: 5.0,
                r: 1,
                g: 1,
                b: 1,
            },
            PointXYZRGB {
                x: 1.3,
                y: 1.6,
                z: 5.7,
                r: 2,
                g: 2,
                b: 2,
            },
            PointXYZRGB {
                x: f32::MAX,
                y: f32::MIN,
                z: f32::MAX,
                r: u8::MAX,
                g: u8::MAX,
                b: u8::MAX,
            },
        ]
    );
}

#[test]
fn converterxyzl() {
    convert_from_into!(
        PointXYZL,
        vec![
            PointXYZL {
                x: 0.0,
                y: 1.0,
                z: 5.0,
                label: 0,
            },
            PointXYZL {
                x: 1.0,
                y: 1.5,
                z: 5.0,
                label: 1,
            },
            PointXYZL {
                x: 1.3,
                y: 1.6,
                z: 5.7,
                label: 2,
            },
            PointXYZL {
                x: f32::MAX,
                y: f32::MIN,
                z: f32::MAX,
                label: u32::MAX,
            },
        ]
    );
}

#[test]
fn converterxyzi() {
    convert_from_into!(
        PointXYZI,
        vec![
            PointXYZI {
                x: 0.0,
                y: 1.0,
                z: 5.0,
                intensity: 0.0,
            },
            PointXYZI {
                x: 1.0,
                y: 1.5,
                z: 5.0,
                intensity: 1.0,
            },
            PointXYZI {
                x: 1.3,
                y: 1.6,
                z: 5.7,
                intensity: 2.0,
            },
            PointXYZI {
                x: f32::MAX,
                y: f32::MIN,
                z: f32::MAX,
                intensity: f32::MAX,
            },
        ]
    );
}

#[test]
fn write_xyzi_read_xyz() {
    let write_cloud = vec![
        PointXYZI {
            x: 0.0,
            y: 1.0,
            z: 5.0,
            intensity: 0.0,
        },
        PointXYZI {
            x: 1.0,
            y: 1.5,
            z: 5.0,
            intensity: 1.0,
        },
        PointXYZI {
            x: 1.3,
            y: 1.6,
            z: 5.7,
            intensity: 2.0,
        },
        PointXYZI {
            x: f32::MAX,
            y: f32::MIN,
            z: f32::MAX,
            intensity: f32::MAX,
        },
    ];

    let read_cloud = [
        PointXYZ {
            x: 0.0,
            y: 1.0,
            z: 5.0,
        },
        PointXYZ {
            x: 1.0,
            y: 1.5,
            z: 5.0,
        },
        PointXYZ {
            x: 1.3,
            y: 1.6,
            z: 5.7,
        },
        PointXYZ {
            x: f32::MAX,
            y: f32::MIN,
            z: f32::MAX,
        },
    ];

    convert_from_into_in_out_cloud!(write_cloud, PointXYZI, read_cloud, PointXYZ);
}

#[test]
fn write_less_than_available() {
    #[derive(Debug, PartialEq, Clone, Default, TypeLayout)]
    #[repr(C)]
    struct CustomPoint {
        x: f32,
        y: f32,
        z: f32,
        dummy: f32,
    }

    impl From<Point<3>> for CustomPoint {
        fn from(point: Point<3>) -> Self {
            Self {
                x: point.fields[0].get(),
                y: point.fields[1].get(),
                z: point.fields[2].get(),
                dummy: 0.0,
            }
        }
    }

    impl From<CustomPoint> for Point<3> {
        fn from(point: CustomPoint) -> Self {
            Point {
                fields: [point.x.into(), point.y.into(), point.z.into()],
            }
        }
    }

    impl Fields<3> for CustomPoint {
        fn field_names_ordered() -> [&'static str; 3] {
            ["x", "y", "z"]
        }
    }

    impl PointConvertible<3> for CustomPoint {}

    let write_cloud = vec![
        CustomPoint {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            dummy: -10.0,
        },
        CustomPoint {
            x: 4.0,
            y: 5.0,
            z: 6.0,
            dummy: -10.0,
        },
        CustomPoint {
            x: 7.0,
            y: 8.0,
            z: 9.0,
            dummy: -10.0,
        },
    ];

    let read_cloud = [
        CustomPoint {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            dummy: 0.0,
        },
        CustomPoint {
            x: 4.0,
            y: 5.0,
            z: 6.0,
            dummy: 0.0,
        },
        CustomPoint {
            x: 7.0,
            y: 8.0,
            z: 9.0,
            dummy: 0.0,
        },
    ];

    convert_from_into_in_out_cloud!(write_cloud, CustomPoint, read_cloud, CustomPoint);
}
