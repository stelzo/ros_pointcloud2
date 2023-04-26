use ros_pointcloud2::{ConversionError, Convert, ConvertXYZ, ConvertXYZINormal, ConvertXYZRGBA, ConvertXYZRGBNormal, MetaNames, PointConvertible, PointMeta};
use ros_pointcloud2::mem_macros::size_of;
use ros_pointcloud2::pcl_utils::{PointXYZ, PointXYZINormal, PointXYZRGBA, PointXYZRGBNormal};
use fallible_iterator::FallibleIterator;


#[test]
fn custom_xyz_f32() {
    const DIM : usize = 3;
    const METADIM : usize = 0;

    #[derive(Debug, PartialEq, Clone)]
    struct CustomPoint {
        x: f32,
        y: f32,
        z: f32,
    }
    type MyConverter = Convert<f32, { size_of!(f32) }, DIM, METADIM, CustomPoint>;
    impl Into<([f32; DIM], [PointMeta; METADIM])> for CustomPoint {
        fn into(self) -> ([f32; DIM], [PointMeta; METADIM]) {
            ([self.x, self.y, self.z], [])
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
    impl MetaNames<METADIM> for CustomPoint {
        fn meta_names() -> [String; METADIM] {
            []
        }
    }
    impl PointConvertible<f32, { size_of!(f32) }, DIM, METADIM> for CustomPoint {}

    let custom_cloud = vec![
        CustomPoint { x: 1.0, y: 2.0, z: 3.0 },
        CustomPoint { x: 4.0, y: 5.0, z: 6.0 },
        CustomPoint { x: 7.0, y: 8.0, z: 9.0 },
    ];
    let copy = custom_cloud.clone();
    let custom_msg: Result<ros_pointcloud2::ros_types::PointCloud2Msg, _> = MyConverter::try_from(custom_cloud).unwrap().try_into();
    assert!(custom_msg.is_ok());
    let to_custom_type = MyConverter::try_from(custom_msg.unwrap());
    assert!(to_custom_type.is_ok());
    let to_custom_type = to_custom_type.unwrap().map(|point| Ok(point)).collect::<Vec<CustomPoint>>();
    assert_eq!(copy, to_custom_type.unwrap());
}


#[test]
fn custom_xyzi_f32() {
    const DIM : usize = 3;
    const METADIM : usize = 1;
    #[derive(Debug, PartialEq, Clone)]
    struct CustomPoint {
        x: f32,
        y: f32,
        z: f32,
        i: u8,
    }
    type MyConverter = Convert<f32, { size_of!(f32) }, DIM, METADIM, CustomPoint>;
    impl Into<([f32; DIM], [PointMeta; METADIM])> for CustomPoint {
        fn into(self) -> ([f32; DIM], [PointMeta; METADIM]) {
            ([self.x, self.y, self.z], [PointMeta::new(self.i)])
        }
    }
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
    impl MetaNames<METADIM> for CustomPoint {
        fn meta_names() -> [String; METADIM] {
            ["intensity"].map(|s| s.to_string())
        }
    }
    impl PointConvertible<f32, { size_of!(f32) }, DIM, METADIM> for CustomPoint {}
    let custom_cloud = vec![
        CustomPoint { x: 0.0, y: 1.0, z: 5.0, i: 0 },
        CustomPoint { x: 1.0, y: 1.5, z: 5.0, i: 1 },
        CustomPoint { x: 1.3, y: 1.6, z: 5.7, i: 2 },
        CustomPoint { x: f32::MAX, y: f32::MIN, z: f32::MAX, i: u8::MAX },
    ];
    let copy = custom_cloud.clone();
    let custom_msg: Result<ros_pointcloud2::ros_types::PointCloud2Msg, _> = MyConverter::try_from(custom_cloud).unwrap().try_into();
    assert!(custom_msg.is_ok());
    let to_custom_type = MyConverter::try_from(custom_msg.unwrap());
    assert!(to_custom_type.is_ok());
    let to_custom_type = to_custom_type.unwrap();
    let back_to_type = to_custom_type.map(|point| Ok(point)).collect::<Vec<CustomPoint>>();
    assert_eq!(copy, back_to_type.unwrap());
}

#[test]
fn custom_rgba_f32() {
    const DIM : usize = 3;
    const METADIM : usize = 4;
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
    type MyConverter = Convert<f32, { size_of!(f32) }, DIM, METADIM, CustomPoint>;
    impl Into<([f32; DIM], [PointMeta; METADIM])> for CustomPoint {
        fn into(self) -> ([f32; DIM], [PointMeta; METADIM]) {
            ([self.x, self.y, self.z], [PointMeta::new(self.r), PointMeta::new(self.g), PointMeta::new(self.b), PointMeta::new(self.a)])
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
    impl MetaNames<METADIM> for CustomPoint {
        fn meta_names() -> [String; METADIM] {
            ["r", "g", "b", "a"].map(|s| s.to_string())
        }
    }
    impl PointConvertible<f32, { size_of!(f32) }, DIM, METADIM> for CustomPoint {}
    let custom_cloud = vec![
        CustomPoint { x: 0.0, y: 1.0, z: 5.0, r: 0, g: 0, b: 0, a: 0 },
        CustomPoint { x: 1.0, y: 1.5, z: 5.0, r: 1, g: 1, b: 1, a: 1 },
        CustomPoint { x: 1.3, y: 1.6, z: 5.7, r: 2, g: 2, b: 2, a: 2 },
        CustomPoint { x: f32::MAX, y: f32::MIN, z: f32::MAX, r: u8::MAX, g: u8::MAX, b: u8::MAX, a: u8::MAX },
    ];
    let copy = custom_cloud.clone();
    let custom_msg: Result<ros_pointcloud2::ros_types::PointCloud2Msg, _> = MyConverter::try_from(custom_cloud).unwrap().try_into();
    assert!(custom_msg.is_ok());
    let to_custom_type = MyConverter::try_from(custom_msg.unwrap());
    assert!(to_custom_type.is_ok());
    let to_custom_type = to_custom_type.unwrap();
    let back_to_type = to_custom_type.map(|point| Ok(point)).collect::<Vec<CustomPoint>>();
    assert_eq!(copy, back_to_type.unwrap());
}

#[test]
fn converterxyz() {
    let cloud = vec![
        PointXYZ { x: 0.0, y: 1.0, z: 5.0 },
        PointXYZ { x: 1.0, y: 1.5, z: 5.0 },
        PointXYZ { x: 1.3, y: 1.6, z: 5.7 },
        PointXYZ { x: f32::MAX, y: f32::MIN, z: f32::MAX },
    ];

    let copy = cloud.clone();
    let msg: Result<ros_pointcloud2::ros_types::PointCloud2Msg, _> = ConvertXYZ::try_from(cloud).unwrap().try_into();
    assert!(msg.is_ok());
    let to_xyz_type = ConvertXYZ::try_from(msg.unwrap());
    assert!(to_xyz_type.is_ok());
    let to_xyz_type: ConvertXYZ = to_xyz_type.unwrap();
    let back_to_type = to_xyz_type.map(|point: PointXYZ| Ok(point)).collect::<Vec<PointXYZ>>();
    assert!(back_to_type.is_ok());
    assert_eq!(copy, back_to_type.unwrap());
}

#[test]
fn converterxyzrgba() {
    let cloud = vec![
        PointXYZRGBA { x: 0.0, y: 1.0, z: 5.0, r: 0, g: 0, b: 0, a: 0 },
        PointXYZRGBA { x: 1.0, y: 1.5, z: 5.0, r: 1, g: 1, b: 1, a: 1 },
        PointXYZRGBA { x: 1.3, y: 1.6, z: 5.7, r: 2, g: 2, b: 2, a: 2 },
        PointXYZRGBA { x: f32::MAX, y: f32::MIN, z: f32::MAX, r: u8::MAX, g: u8::MAX, b: u8::MAX, a: u8::MAX },
    ];

    let copy = cloud.clone();
    let msg: Result<ros_pointcloud2::ros_types::PointCloud2Msg, _> = ConvertXYZRGBA::try_from(cloud).unwrap().try_into();
    assert!(msg.is_ok());
    let to_xyzrgba_type = ConvertXYZRGBA::try_from(msg.unwrap());
    assert!(to_xyzrgba_type.is_ok());
    let to_xyzrgba_type = to_xyzrgba_type.unwrap();
    let back_to_type = to_xyzrgba_type.map(|point| Ok(point)).collect::<Vec<PointXYZRGBA>>();
    assert_eq!(copy, back_to_type.unwrap());
}

#[test]
fn converterxyzinormal() {
    let cloud = vec![
        PointXYZINormal { x: 0.0, y: 1.0, z: 5.0, intensity: 0.0, normal_x: 0.0, normal_y: 0.0, normal_z: 0.0 },
        PointXYZINormal { x: 1.0, y: 1.5, z: 5.0, intensity: 1.0, normal_x: 1.0, normal_y: 1.0, normal_z: 1.0 },
        PointXYZINormal { x: 1.3, y: 1.6, z: 5.7, intensity: 2.0, normal_x: 2.0, normal_y: 2.0, normal_z: 2.0 },
        PointXYZINormal { x: f32::MAX, y: f32::MIN, z: f32::MAX, intensity: f32::MAX, normal_x: f32::MAX, normal_y: f32::MAX, normal_z: f32::MAX },
    ];

    let copy = cloud.clone();
    let msg: Result<ros_pointcloud2::ros_types::PointCloud2Msg, _> = ConvertXYZINormal::try_from(cloud).unwrap().try_into();
    assert!(msg.is_ok());
    let to_xyzinormal_type = ConvertXYZINormal::try_from(msg.unwrap());
    assert!(to_xyzinormal_type.is_ok());
    let to_xyzinormal_type = to_xyzinormal_type.unwrap();
    let back_to_type = to_xyzinormal_type.map(|point| Ok(point)).collect::<Vec<PointXYZINormal>>();
    assert_eq!(copy, back_to_type.unwrap());
}

#[test]
fn converterxyzrgbnormal() {
    let cloud = vec![
        PointXYZRGBNormal { x: 0.0, y: 1.0, z: 5.0, r: 0, g: 0, b: 0, normal_x: 0.0, normal_y: 0.0, normal_z: 0.0 },
        PointXYZRGBNormal { x: 1.0, y: 1.5, z: 5.0, r: 1, g: 1, b: 1, normal_x: 1.0, normal_y: 1.0, normal_z: 1.0 },
        PointXYZRGBNormal { x: 1.3, y: 1.6, z: 5.7, r: 2, g: 2, b: 2, normal_x: 2.0, normal_y: 2.0, normal_z: 2.0 },
        PointXYZRGBNormal { x: f32::MAX, y: f32::MIN, z: f32::MAX, r: u8::MAX, g: u8::MAX, b: u8::MAX, normal_x: f32::MAX, normal_y: f32::MAX, normal_z: f32::MAX },
    ];

    let copy = cloud.clone();
    let msg: Result<ros_pointcloud2::ros_types::PointCloud2Msg, _> = ConvertXYZRGBNormal::try_from(cloud).unwrap().try_into();
    assert!(msg.is_ok());
    let to_xyzrgbnormal_type = ConvertXYZRGBNormal::try_from(msg.unwrap());
    assert!(to_xyzrgbnormal_type.is_ok());
    let to_xyzrgbnormal_type = to_xyzrgbnormal_type.unwrap();
    let back_to_type = to_xyzrgbnormal_type.map(|point| Ok(point)).collect::<Vec<PointXYZRGBNormal>>();
    assert_eq!(copy, back_to_type.unwrap());
}
