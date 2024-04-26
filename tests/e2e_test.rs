use fallible_iterator::FallibleIterator;
use ros_pointcloud2::pcl_utils::*;
use ros_pointcloud2::reader::*;
use ros_pointcloud2::ros_types::PointCloud2Msg;
use ros_pointcloud2::writer::*;
use ros_pointcloud2::*;
use std::fmt::Debug;

macro_rules! convert_from_into {
    ($reader:ty, $writer:ty, $point:ty, $cloud:expr) => {
        let copy = $cloud.clone();
        let msg: Result<PointCloud2Msg, _> = <$writer>::from($cloud).try_into();
        assert!(msg.is_ok());
        let to_p_type = <$reader>::try_from(msg.unwrap());
        assert!(to_p_type.is_ok());
        let to_p_type = to_p_type.unwrap();
        let back_to_type = to_p_type.map(|point| Ok(point)).collect::<Vec<$point>>();
        assert_eq!(copy, back_to_type.unwrap());
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
    type MyReader = Reader<f32, { size_of!(f32) }, DIM, METADIM, CustomPoint>;
    type MyWriter = Writer<f32, { size_of!(f32) }, DIM, METADIM, CustomPoint>;
    impl From<CustomPoint> for ([f32; DIM], [PointMeta; METADIM]) {
        fn from(point: CustomPoint) -> Self {
            ([point.x, point.y, point.z], [])
        }
    }
    impl TryFrom<([f32; DIM], [PointMeta; METADIM])> for CustomPoint {
        type Error = ConversionError;
        fn try_from(data: ([f32; DIM], [PointMeta; METADIM])) -> Result<Self, ConversionError> {
            Ok(Self {
                x: data.0[0],
                y: data.0[1],
                z: data.0[2],
            })
        }
    }
    impl convert::MetaNames<METADIM> for CustomPoint {
        fn meta_names() -> [String; METADIM] {
            []
        }
    }
    impl PointConvertible<f32, { size_of!(f32) }, DIM, METADIM> for CustomPoint {}

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
    const DIM: usize = 3;
    const METADIM: usize = 1;
    #[derive(Debug, PartialEq, Clone)]
    struct CustomPoint {
        x: f32,
        y: f32,
        z: f32,
        i: u8,
    }
    type MyReader = reader::Reader<f32, { size_of!(f32) }, DIM, METADIM, CustomPoint>;
    impl From<CustomPoint> for ([f32; DIM], [PointMeta; METADIM]) {
        fn from(point: CustomPoint) -> Self {
            ([point.x, point.y, point.z], [PointMeta::new(point.i)])
        }
    }

    type MyWriter = writer::Writer<f32, { size_of!(f32) }, DIM, METADIM, CustomPoint>;

    impl TryFrom<([f32; DIM], [PointMeta; METADIM])> for CustomPoint {
        type Error = ConversionError;
        fn try_from(data: ([f32; DIM], [PointMeta; METADIM])) -> Result<Self, ConversionError> {
            Ok(Self {
                x: data.0[0],
                y: data.0[1],
                z: data.0[2],
                i: data.1.first().unwrap().get().unwrap(),
            })
        }
    }
    impl convert::MetaNames<METADIM> for CustomPoint {
        fn meta_names() -> [String; METADIM] {
            [format!("intensity")]
        }
    }
    impl PointConvertible<f32, { size_of!(f32) }, DIM, METADIM> for CustomPoint {}

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
    type MyReader = reader::Reader<f32, { size_of!(f32) }, DIM, METADIM, CustomPoint>;
    type MyWriter = writer::Writer<f32, { size_of!(f32) }, DIM, METADIM, CustomPoint>;
    impl From<CustomPoint> for ([f32; DIM], [PointMeta; METADIM]) {
        fn from(point: CustomPoint) -> Self {
            (
                [point.x, point.y, point.z],
                [
                    PointMeta::new(point.r),
                    PointMeta::new(point.g),
                    PointMeta::new(point.b),
                    PointMeta::new(point.a),
                ],
            )
        }
    }
    impl TryFrom<([f32; DIM], [PointMeta; METADIM])> for CustomPoint {
        type Error = ConversionError;
        fn try_from(data: ([f32; DIM], [PointMeta; METADIM])) -> Result<Self, Self::Error> {
            Ok(Self {
                x: data.0[0],
                y: data.0[1],
                z: data.0[2],
                r: data.1[0].get()?,
                g: data.1[1].get()?,
                b: data.1[2].get()?,
                a: data.1[3].get()?,
            })
        }
    }
    impl convert::MetaNames<METADIM> for CustomPoint {
        fn meta_names() -> [String; METADIM] {
            ["r", "g", "b", "a"].map(|s| s.to_string())
        }
    }
    impl PointConvertible<f32, { size_of!(f32) }, DIM, METADIM> for CustomPoint {}
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
