use ros_pointcloud2::pcl_utils::*;
use ros_pointcloud2::reader::*;
use ros_pointcloud2::ros_types::PointCloud2Msg;
use ros_pointcloud2::writer::*;
use ros_pointcloud2::*;
use std::fmt::Debug;

macro_rules! convert_from_into {
    ($reader:ty, $writer:ty, $point:ty, $cloud:expr) => {
        convert_from_into_in_out_cloud!(
            $reader,
            $writer,
            $point,
            $cloud.clone(),
            $point,
            $cloud,
            $point
        );
    };
}

macro_rules! convert_from_into_in_out_cloud {
    ($reader:ty, $writer:ty, $point:ty, $in_cloud:expr, $in_point:ty, $out_cloud:expr, $out_point:ty) => {
        let msg: Result<PointCloud2Msg, _> = <$writer>::from($in_cloud).try_into();
        assert!(msg.is_ok());
        let to_p_type = <$reader>::try_from(msg.unwrap());
        assert!(to_p_type.is_ok());
        let to_p_type = to_p_type.unwrap();
        let back_to_type = to_p_type.collect::<Vec<$out_point>>();
        assert_eq!($out_cloud, back_to_type);
    };
}

#[test]
fn custom_xyz_f32() {
    const DIM: usize = 3;
    const METADIM: usize = 0;

    #[derive(Debug, PartialEq, Clone)]
    struct CustomPoint {
        x: f32,
        y: f32,
        z: f32,
    }

    type MyReader = ReaderF32<DIM, METADIM, CustomPoint>;
    type MyWriter = WriterF32<DIM, METADIM, CustomPoint>;

    impl From<Point<f32, 3, 0>> for CustomPoint {
        fn from(point: Point<f32, 3, 0>) -> Self {
            Self {
                x: point.coords[0],
                y: point.coords[1],
                z: point.coords[2],
            }
        }
    }

    impl From<CustomPoint> for Point<f32, 3, 0> {
        fn from(point: CustomPoint) -> Self {
            Point {
                coords: [point.x, point.y, point.z],
                meta: [],
            }
        }
    }

    impl MetaNames<METADIM> for CustomPoint {
        fn meta_names() -> [&'static str; METADIM] {
            []
        }
    }
    impl PointConvertible<f32, { std::mem::size_of::<f32>() }, 3, 0> for CustomPoint {}

    convert_from_into!(
        MyReader,
        MyWriter,
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
fn custom_xyzi_f32() {
    type Xyz = f32;
    const XYZ_S: usize = std::mem::size_of::<Xyz>();
    const DIM: usize = 3;
    const METADIM: usize = 1;
    #[derive(Debug, PartialEq, Clone)]
    struct CustomPoint {
        x: f32,
        y: f32,
        z: f32,
        i: u8,
    }

    impl From<Point<f32, 3, 1>> for CustomPoint {
        fn from(point: Point<f32, 3, 1>) -> Self {
            Self {
                x: point.coords[0],
                y: point.coords[1],
                z: point.coords[2],
                i: point.meta[0].get(),
            }
        }
    }

    impl From<CustomPoint> for Point<f32, 3, 1> {
        fn from(point: CustomPoint) -> Self {
            Point {
                coords: [point.x, point.y, point.z],
                meta: [point.i.into()],
            }
        }
    }

    impl MetaNames<METADIM> for CustomPoint {
        fn meta_names() -> [&'static str; METADIM] {
            ["intensity"]
        }
    }

    type MyReader = reader::Reader<Xyz, XYZ_S, DIM, METADIM, CustomPoint>;
    type MyWriter = writer::Writer<Xyz, XYZ_S, DIM, METADIM, CustomPoint>;

    impl PointConvertible<Xyz, XYZ_S, DIM, METADIM> for CustomPoint {}

    let cloud = vec![
        CustomPoint {
            x: 0.0,
            y: 1.0,
            z: 5.0,
            i: 0,
        },
        CustomPoint {
            x: 1.0,
            y: 1.5,
            z: 5.0,
            i: 1,
        },
        CustomPoint {
            x: 1.3,
            y: 1.6,
            z: 5.7,
            i: 2,
        },
        CustomPoint {
            x: f32::MAX,
            y: f32::MIN,
            z: f32::MAX,
            i: u8::MAX,
        },
    ];
    convert_from_into!(MyReader, MyWriter, CustomPoint, cloud);
}

#[test]
fn custom_rgba_f32() {
    const DIM: usize = 3;
    const METADIM: usize = 4;
    #[derive(Debug, PartialEq, Clone)]
    struct CustomPoint {
        x: f32,
        y: f32,
        z: f32,
        r: u8,
        g: u8,
        b: u8,
        a: u8,
    }
    type MyReader = reader::Reader<f32, { std::mem::size_of::<f32>() }, DIM, METADIM, CustomPoint>;
    type MyWriter = writer::Writer<f32, { std::mem::size_of::<f32>() }, DIM, METADIM, CustomPoint>;

    impl From<Point<f32, 3, 4>> for CustomPoint {
        fn from(point: Point<f32, 3, 4>) -> Self {
            Self {
                x: point.coords[0],
                y: point.coords[1],
                z: point.coords[2],
                r: point.meta[0].get(),
                g: point.meta[1].get(),
                b: point.meta[2].get(),
                a: point.meta[3].get(),
            }
        }
    }

    impl From<CustomPoint> for Point<f32, 3, 4> {
        fn from(point: CustomPoint) -> Self {
            Point {
                coords: [point.x, point.y, point.z],
                meta: [
                    point.r.into(),
                    point.g.into(),
                    point.b.into(),
                    point.a.into(),
                ],
            }
        }
    }

    impl MetaNames<METADIM> for CustomPoint {
        fn meta_names() -> [&'static str; METADIM] {
            ["r", "g", "b", "a"]
        }
    }
    impl PointConvertible<f32, { std::mem::size_of::<f32>() }, DIM, METADIM> for CustomPoint {}
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
    convert_from_into!(MyReader, MyWriter, CustomPoint, cloud);
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

    convert_from_into!(ReaderXYZ, WriterXYZ, PointXYZ, cloud);
}

#[test]
fn converterxyzrgba() {
    convert_from_into!(
        ReaderXYZRGBA,
        WriterXYZRGBA,
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
        ReaderXYZINormal,
        WriterXYZINormal,
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
        ReaderXYZRGBNormal,
        WriterXYZRGBNormal,
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
        ReaderXYZNormal,
        WriterXYZNormal,
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
        ReaderXYZRGBL,
        WriterXYZRGBL,
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
        ReaderXYZRGB,
        WriterXYZRGB,
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
        ReaderXYZL,
        WriterXYZL,
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
        ReaderXYZI,
        WriterXYZI,
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

    let read_cloud = vec![
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

    convert_from_into_in_out_cloud!(
        ReaderXYZ,
        WriterXYZI,
        PointXYZI,
        write_cloud,
        PointXYZI,
        read_cloud,
        PointXYZ
    );
}

#[test]
fn write_less_than_available() {
    const DIM: usize = 3;
    const METADIM: usize = 0;

    #[derive(Debug, PartialEq, Clone)]
    struct CustomPoint {
        x: f32,
        y: f32,
        z: f32,
        dummy: f32,
    }

    type MyReader = ReaderF32<DIM, METADIM, CustomPoint>;
    type MyWriter = WriterF32<DIM, METADIM, CustomPoint>;

    impl From<Point<f32, 3, 0>> for CustomPoint {
        fn from(point: Point<f32, 3, 0>) -> Self {
            Self {
                x: point.coords[0],
                y: point.coords[1],
                z: point.coords[2],
                dummy: 0.0,
            }
        }
    }

    impl From<CustomPoint> for Point<f32, 3, 0> {
        fn from(point: CustomPoint) -> Self {
            Point {
                coords: [point.x, point.y, point.z],
                meta: [],
            }
        }
    }

    impl MetaNames<METADIM> for CustomPoint {
        fn meta_names() -> [&'static str; METADIM] {
            []
        }
    }
    impl PointConvertible<f32, { std::mem::size_of::<f32>() }, 3, 0> for CustomPoint {}

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

    let read_cloud = vec![
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

    convert_from_into_in_out_cloud!(
        MyReader,
        MyWriter,
        CustomPoint,
        write_cloud,
        CustomPoint,
        read_cloud,
        CustomPoint
    );
}

#[test]
fn readme() {
    use ros_pointcloud2::{
        pcl_utils::PointXYZ, reader::ReaderXYZ, writer::WriterXYZ, PointCloud2Msg,
    };

    // Your points (here using the predefined type PointXYZ).
    let cloud_points = vec![
        PointXYZ {
            x: 1337.0,
            y: 42.0,
            z: 69.0,
        },
        PointXYZ {
            x: f32::MAX,
            y: f32::MIN,
            z: f32::MAX,
        },
    ];

    // For equality test later
    let cloud_copy = cloud_points.clone();

    // Vector -> Writer -> Message
    let internal_msg: PointCloud2Msg = WriterXYZ::from(cloud_points)
        .try_into() // iterating points here O(n)
        .unwrap();

    // Convert to your ROS crate message type, we will use r2r here.
    // let msg: r2r::sensor_msgs::msg::PointCloud2 = internal_msg.into();

    // Publish ...

    // ... now incoming from a topic.
    // let internal_msg: PointCloud2Msg = msg.into();

    // Message -> Reader. The Reader implements the Iterator trait.
    let reader = ReaderXYZ::try_from(internal_msg).unwrap();
    let new_cloud_points = reader
        .map(|point: PointXYZ| {
            // Some logic here

            point
        })
        .collect::<Vec<PointXYZ>>();

    assert_eq!(new_cloud_points, cloud_copy);
}
