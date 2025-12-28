#[cfg(test)]
mod test {
    #![allow(clippy::unwrap_used)]

    use crate::prelude::*;
    use crate::{ByteSimilarity, PointData};
    use alloc::string::{String, ToString};
    use alloc::vec::Vec;

    #[derive(Debug, Default, Clone, PartialEq, Copy)]
    #[repr(C)]
    struct PointA {
        x: f32,
        y: f32,
        z: f32,
        intensity: f32,
        t: u32,
        reflectivity: u16,
        ring: u16,
        ambient: u16,
        range: u32,
    }

    impl From<IPoint<9>> for PointA {
        fn from(point: IPoint<9>) -> Self {
            Self::new(point[0].get(), point[1].get(), point[2].get())
        }
    }

    impl From<PointA> for IPoint<9> {
        fn from(point: PointA) -> Self {
            [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                point.intensity.into(),
                point.t.into(),
                point.reflectivity.into(),
                point.ring.into(),
                point.ambient.into(),
                point.range.into(),
            ]
            .into()
        }
    }

    unsafe impl PointConvertible<9> for PointA {
        fn layout() -> LayoutDescription {
            LayoutDescription::new(&[
                LayoutField::new("x", "f32", 4),
                LayoutField::new("y", "f32", 4),
                LayoutField::new("z", "f32", 4),
                LayoutField::new("intensity", "f32", 4),
                LayoutField::new("t", "u32", 4),
                LayoutField::new("reflectivity", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("ring", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("ambient", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("range", "u32", 4),
            ])
        }
    }

    impl PointA {
        fn new(x: f32, y: f32, z: f32) -> Self {
            Self {
                x,
                y,
                z,
                intensity: 0.0,
                t: 0,
                reflectivity: 0,
                ring: 0,
                ambient: 0,
                range: 0,
            }
        }
    }

    #[derive(Debug, Clone, Default, PartialEq, Copy)]
    #[repr(C)]
    struct PointB {
        pub x: f32,
        pub y: f32,
        pub z: f32,
        pub t: u32,
    }

    impl PointB {
        fn new(x: f32, y: f32, z: f32) -> Self {
            Self { x, y, z, t: 0 }
        }
    }

    impl From<IPoint<4>> for PointB {
        fn from(point: IPoint<4>) -> Self {
            Self::new(point[0].get(), point[1].get(), point[2].get())
        }
    }

    impl From<PointB> for IPoint<4> {
        fn from(point: PointB) -> Self {
            [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                point.t.into(),
            ]
            .into()
        }
    }

    unsafe impl PointConvertible<4> for PointB {
        fn layout() -> LayoutDescription {
            LayoutDescription::new(&[
                LayoutField::new("x", "f32", 4),
                LayoutField::new("y", "f32", 4),
                LayoutField::new("z", "f32", 4),
                LayoutField::new("t", "u32", 4),
            ])
        }
    }

    #[derive(Debug, Clone, Default, PartialEq, Copy)]
    #[repr(C)]
    struct PointD {
        x: f32,
        y: f32,
        z: f32,
        t: u32,
        ring: u16,
        range: u32,
        signal: u16,
        reflectivity: u16,
        near_ir: u16,
    }

    impl From<IPoint<9>> for PointD {
        fn from(point: IPoint<9>) -> Self {
            Self::new(point[0].get(), point[1].get(), point[2].get())
        }
    }

    impl From<PointD> for IPoint<9> {
        fn from(point: PointD) -> Self {
            [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                point.t.into(),
                point.ring.into(),
                point.range.into(),
                point.signal.into(),
                point.reflectivity.into(),
                point.near_ir.into(),
            ]
            .into()
        }
    }

    unsafe impl PointConvertible<9> for PointD {
        fn layout() -> LayoutDescription {
            LayoutDescription::new(&[
                LayoutField::new("x", "f32", 4),
                LayoutField::new("y", "f32", 4),
                LayoutField::new("z", "f32", 4),
                LayoutField::new("t", "u32", 4),
                LayoutField::new("ring", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("range", "u32", 4),
                LayoutField::new("signal", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("reflectivity", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("near_ir", "u16", 2),
                LayoutField::padding(2),
            ])
        }
    }

    impl PointD {
        fn new(x: f32, y: f32, z: f32) -> Self {
            Self {
                x,
                y,
                z,
                t: 0,
                ring: 0,
                range: 0,
                signal: 0,
                reflectivity: 0,
                near_ir: 0,
            }
        }
    }

    #[test]
    fn subtype_iterator_fallback() {
        let cloud_a = PointCloud2Msg::try_from_iter(&vec![
            PointA::new(1.0, 2.0, 3.0),
            PointA::new(4.0, 5.0, 6.0),
            PointA::new(7.0, 8.0, 9.0),
        ])
        .unwrap();

        let cloud_c: PointB = cloud_a.try_into_iter().unwrap().next().unwrap();
        assert_eq!(cloud_c, PointB::new(1.0, 2.0, 3.0));

        let cloud_b: Vec<PointB> = cloud_a.try_into_vec().unwrap();
        assert_eq!(cloud_b[0], PointB::new(1.0, 2.0, 3.0));
        assert_eq!(cloud_b[1], PointB::new(4.0, 5.0, 6.0));
        assert_eq!(cloud_b[2], PointB::new(7.0, 8.0, 9.0));
    }

    #[test]
    fn byte_similarity_equal() {
        let pts = vec![PointB::new(1.0, 2.0, 3.0), PointB::new(4.0, 5.0, 6.0)];
        let msg = PointCloud2Msg::try_from_slice(&pts).unwrap();
        assert_eq!(
            msg.byte_similarity::<4, PointB>().unwrap(),
            ByteSimilarity::Equal
        );
        let out: Vec<PointB> = msg.try_into_vec().unwrap();
        assert_eq!(out, pts);
    }

    #[test]
    fn byte_similarity_overlapping() {
        let pts = vec![PointB::new(1.0, 2.0, 3.0), PointB::new(4.0, 5.0, 6.0)];
        let base = PointCloud2Msg::try_from_slice(&pts).unwrap();
        let old_step = base.point_step as usize;
        let new_step = old_step + 4;
        let mut new_data = Vec::with_capacity((base.data.len() / old_step) * new_step);
        base.data.chunks(old_step).for_each(|chunk| {
            new_data.extend_from_slice(chunk);
            new_data.extend_from_slice(&[0; 4]);
        });
        let mut msg = base.clone();
        msg.point_step = new_step as u32;
        msg.row_step = (pts.len() as u32) * (new_step as u32);
        msg.data = new_data;
        assert_eq!(
            msg.byte_similarity::<4, PointB>().unwrap(),
            ByteSimilarity::Overlapping
        );
        let out: Vec<PointB> = msg.try_into_vec().unwrap();
        assert_eq!(out, pts);
    }

    #[test]
    fn byte_similarity_different() {
        let cloud_a = PointCloud2Msg::try_from_iter(&vec![
            PointA::new(1.0, 2.0, 3.0),
            PointA::new(4.0, 5.0, 6.0),
        ])
        .unwrap();
        assert_eq!(
            cloud_a.byte_similarity::<4, PointB>().unwrap(),
            ByteSimilarity::Different
        );
        let out: Vec<PointB> = cloud_a.try_into_vec().unwrap();
        assert_eq!(out[0], PointB::new(1.0, 2.0, 3.0));
        assert_eq!(out[1], PointB::new(4.0, 5.0, 6.0));
    }

    #[cfg(feature = "strict-type-check")]
    #[test]
    fn strict_disallow_mismatched_same_size() {
        let pdata = PointData::new(42i32);
        let res: Result<f32, MsgConversionError> = pdata.get_checked();
        assert!(matches!(res, Err(MsgConversionError::TypeMismatch { .. })));
    }

    #[cfg(feature = "strict-type-check")]
    #[test]
    fn strict_allow_rgb_f32() {
        let rgb = crate::points::RGB::new(1, 2, 3);
        let pdata = PointData::new(rgb);
        let f_res: Result<f32, MsgConversionError> = pdata.get_checked();
        assert!(f_res.is_ok());
        let _f = f_res.unwrap();
        let rgb2: crate::points::RGB = pdata.get_checked().unwrap();
        assert_eq!(rgb.r(), rgb2.r());
    }

    #[cfg(not(feature = "strict-type-check"))]
    #[test]
    fn non_strict_allows_mismatch() {
        let pdata = PointData::new(42i32);
        let _: f32 = pdata.get(); // should not panic when feature is disabled
    }

    #[test]
    fn try_from_vec_deprecated_wrapper_forwards() {
        let pts = vec![PointB::new(1.0, 2.0, 3.0), PointB::new(4.0, 5.0, 6.0)];
        let msg_a = PointCloud2Msg::try_from_slice(&pts).unwrap();
        #[expect(deprecated)]
        let msg_b = PointCloud2Msg::try_from_vec(&pts).unwrap();
        assert_eq!(msg_a.data, msg_b.data);
        assert_eq!(msg_a.point_step, msg_b.point_step);
        assert_eq!(msg_a.fields.len(), msg_b.fields.len());
        for (fa, fb) in msg_a.fields.iter().zip(msg_b.fields.iter()) {
            assert_eq!(fa.name, fb.name);
            assert_eq!(fa.offset, fb.offset);
            assert_eq!(fa.datatype, fb.datatype);
            assert_eq!(fa.count, fb.count);
        }
        assert_eq!(msg_a.row_step, msg_b.row_step);
    }

    #[test]
    fn msg_conversion_error_is_core_error() {
        let e = MsgConversionError::NumberConversion;
        let _err_obj: &dyn core::error::Error = &e;
    }

    #[test]
    fn fields_not_found_display_contains_fields_in_unit_tests() {
        let fields: Vec<String> = vec!["x".to_string(), "y".to_string()];
        let err = MsgConversionError::FieldsNotFound(fields.clone());
        let s = format!("{}", err);
        assert!(s.contains("Some fields are not found"));
        assert!(s.contains("x") && s.contains("y"));
    }

    #[test]
    fn iterator_returns_fields_not_found_when_missing_multiple_unit_tests() {
        let pts = vec![PointB::new(1.0, 2.0, 3.0)];
        let mut msg = PointCloud2Msg::try_from_slice(&pts).unwrap();

        // remove the "y" and "t" fields to trigger FieldsNotFound for the PointB layout
        let mut positions: Vec<usize> = msg
            .fields
            .iter()
            .enumerate()
            .filter(|(_, f)| f.name == "y" || f.name == "t")
            .map(|(i, _)| i)
            .collect();
        // remove descending so indexes remain valid
        positions.sort_unstable_by(|a, b| b.cmp(a));
        for p in positions {
            msg.fields.remove(p);
        }

        let res = crate::iterator::PointCloudIterator::<4, PointB>::try_from(&msg);
        match res {
            Err(MsgConversionError::FieldsNotFound(names)) => {
                assert!(names.contains(&"y".to_string()));
                assert!(names.contains(&"t".to_string()));
            }
            _ => panic!("Expected FieldsNotFound error"),
        }
    }

    #[cfg(all(feature = "serde", feature = "rkyv"))]
    #[test]
    fn serde_and_rkyv_derive_compat() {
        fn assert_traits<T: serde::Serialize + rkyv::Archive>() {}

        assert_traits::<crate::PointCloud2Msg>();
        assert_traits::<crate::points::PointXYZ>();
    }

    // e2e tests moved from `tests/e2e_test.rs` — they run as crate unit tests now.
    use pretty_assertions::assert_eq;

    #[cfg(feature = "derive")]
    use std::fmt::Debug;

    macro_rules! convert_from_into {
        ($point:ty, $cloud:expr) => {
            convert_from_into_in_out_cloud!($cloud, $point, $cloud, $point);
        };
    }

    macro_rules! convert_from_into_vec {
        ($point:ty, $cloud:expr) => {
            convert_from_into_in_out_cloud_vec!($cloud, $point, $cloud, $point);
        };
    }

    macro_rules! convert_from_into_in_out_cloud {
        ($in_cloud:expr, $in_point:ty, $out_cloud:expr, $out_point:ty) => {
            let msg = PointCloud2Msg::try_from_iter($in_cloud.iter());
            assert!(msg.is_ok(), "{:?}", msg);
            let msg = msg.unwrap();
            let to_p_type = msg.try_into_iter();
            assert!(to_p_type.is_ok());
            let to_p_type = to_p_type.unwrap();
            let back_to_type = to_p_type.collect::<Vec<$out_point>>();
            let orig_cloud: Vec<$out_point> = $out_cloud.iter().cloned().collect();
            assert_eq!(orig_cloud, back_to_type);
        };
    }

    macro_rules! convert_from_into_in_out_cloud_vec {
        ($in_cloud:expr, $in_point:ty, $out_cloud:expr, $out_point:ty) => {
            let msg = PointCloud2Msg::try_from_slice(&$in_cloud);
            assert!(msg.is_ok(), "{:?}", msg);
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
            PointXYZ::new(0.0, 1.0, 5.0),
            PointXYZ::new(1.0, 1.5, 5.0),
            PointXYZ::new(1.3, 1.6, 5.7),
            PointXYZ::new(f32::MAX, f32::MIN, f32::MAX),
        ];

        let msg = PointCloud2Msg::try_from_iter(&cloud);
        assert!(msg.is_ok());
    }

    #[test]
    fn write_cloud_from_vec() {
        let cloud = vec![
            PointXYZ::new(0.0, 1.0, 5.0),
            PointXYZ::new(1.0, 1.5, 5.0),
            PointXYZ::new(1.3, 1.6, 5.7),
            PointXYZ::new(f32::MAX, f32::MIN, f32::MAX),
        ];

        let msg = PointCloud2Msg::try_from_slice(&cloud);
        assert!(msg.is_ok());
    }

    #[test]
    fn write_empty_cloud_vec() {
        let cloud: Vec<PointXYZ> = vec![];
        let msg = PointCloud2Msg::try_from_slice(&cloud);
        assert!(msg.is_ok());
        assert!(msg.unwrap().data.is_empty());
    }

    #[test]
    fn write_empty_cloud_iter() {
        let cloud: Vec<PointXYZ> = vec![];
        let msg = PointCloud2Msg::try_from_iter(&cloud);
        assert!(msg.is_ok());
        assert!(msg.unwrap().data.is_empty());
    }

    #[test]
    #[cfg(feature = "rayon")]
    fn conv_cloud_par_iter() {
        let cloud = vec![
            PointXYZ::new(0.0, 1.0, 5.0),
            PointXYZ::new(1.0, 1.5, 5.0),
            PointXYZ::new(1.3, 1.6, 5.7),
        ];
        let copy = cloud.clone();

        let msg: Result<PointCloud2Msg, MsgConversionError> =
            PointCloud2Msg::try_from_slice(&cloud);
        assert!(msg.is_ok());
        let msg = msg.unwrap();
        let to_p_type = msg.try_into_par_iter();
        assert!(to_p_type.is_ok());
        let to_p_type = to_p_type.unwrap();
        let back_to_type = to_p_type.collect::<Vec<PointXYZ>>();
        assert_eq!(copy, back_to_type);
    }

    #[test]
    #[cfg(feature = "rayon")]
    fn conv_cloud_par_par_iter() {
        let cloud = vec![
            PointXYZ::new(0.0, 1.0, 5.0),
            PointXYZ::new(1.0, 1.5, 5.0),
            PointXYZ::new(1.3, 1.6, 5.7),
            PointXYZ::new(f32::MAX, f32::MIN, f32::MAX),
        ];
        let copy = cloud.clone();

        let msg = PointCloud2Msg::try_from_par_iter(cloud.into_par_iter());
        assert!(msg.is_ok());
        let msg = msg.unwrap();
        let to_p_type = msg.try_into_par_iter();
        assert!(to_p_type.is_ok());
        let to_p_type = to_p_type.unwrap();
        let back_to_type = to_p_type.collect::<Vec<PointXYZ>>();
        assert_eq!(copy, back_to_type);
    }

    #[test]
    #[cfg(feature = "rayon")]
    fn conv_cloud_par_iter_large() {
        let n = 10_000usize;
        let cloud: Vec<PointXYZ> = (0..n)
            .map(|i| PointXYZ::new(i as f32, i as f32 + 0.5, i as f32 + 1.0))
            .collect();
        let copy = cloud.clone();

        let msg = PointCloud2Msg::try_from_slice(&cloud);
        assert!(msg.is_ok());
        let msg = msg.unwrap();
        let to_p_type = msg.try_into_par_iter::<3, PointXYZ>();
        assert!(to_p_type.is_ok());
        let back_to_type = to_p_type.unwrap().collect::<Vec<PointXYZ>>();
        assert_eq!(copy, back_to_type);

        // convenience: (none) — collect via iterator directly
        let msg2 = PointCloud2Msg::try_from_slice(&copy).unwrap();
        let out = msg2
            .try_into_par_iter::<3, PointXYZ>()
            .unwrap()
            .collect::<Vec<_>>();
        assert_eq!(out, copy);
    }

    #[test]
    #[cfg(feature = "rayon")]
    fn conv_cloud_par_iter_endian_mismatch() {
        let cloud = vec![
            PointXYZ::new(0.0, 1.0, 5.0),
            PointXYZ::new(1.0, 1.5, 5.0),
            PointXYZ::new(1.3, 1.6, 5.7),
        ];

        let mut msg = PointCloud2Msg::try_from_slice(&cloud).unwrap();
        // Convert the stored bytes to big-endian representation, then set the endian flag.
        use ros_pointcloud2::{Endian, FieldDatatype};
        if cfg!(target_endian = "little") {
            for i in 0..cloud.len() {
                let base = i * (msg.point_step as usize);
                for f in msg.fields.iter() {
                    let datatype = FieldDatatype::try_from(f).unwrap();
                    let sz = datatype.size();
                    if sz > 1 {
                        let start = base + f.offset as usize;
                        let end = start + sz;
                        msg.data[start..end].reverse();
                    }
                }
            }
        }
        msg.endian = Endian::Big;

        let to_p_type = msg.try_into_par_iter::<3, PointXYZ>();
        assert!(to_p_type.is_ok());
        let back_to_type = to_p_type.unwrap().collect::<Vec<PointXYZ>>();
        assert_eq!(cloud, back_to_type);
    }

    #[test]
    #[cfg(feature = "rayon")]
    fn conv_cloud_par_iter_concurrent() {
        let cloud: Vec<PointXYZ> = (0..1024)
            .map(|i| PointXYZ::new(i as f32, i as f32 + 1.0, i as f32 + 2.0))
            .collect();
        let copy = cloud.clone();

        let msg = PointCloud2Msg::try_from_slice(&cloud).unwrap();

        let it1 = msg.try_into_par_iter::<3, PointXYZ>().unwrap();
        let it2 = msg.try_into_par_iter::<3, PointXYZ>().unwrap();

        let (r1, r2) = rayon::join(|| it1.collect::<Vec<_>>(), || it2.collect::<Vec<_>>());

        assert_eq!(r1, r2);
        assert_eq!(r1, copy);
    }

    #[test]
    #[cfg(feature = "derive")]
    fn custom_xyz_f32() {
        #[derive(Debug, PartialEq, Clone, Default, Copy)]
        #[repr(C, align(4))]
        struct CustomPoint {
            x: f32,
            y: f32,
            z: f32,
        }

        impl From<IPoint<3>> for CustomPoint {
            fn from(point: IPoint<3>) -> Self {
                Self {
                    x: point[0].get(),
                    y: point[1].get(),
                    z: point[2].get(),
                }
            }
        }

        impl From<CustomPoint> for IPoint<3> {
            fn from(point: CustomPoint) -> Self {
                [point.x.into(), point.y.into(), point.z.into()].into()
            }
        }

        unsafe impl PointConvertible<3> for CustomPoint {
            fn layout() -> LayoutDescription {
                LayoutDescription::new(&[
                    LayoutField::new("x", "f32", 4),
                    LayoutField::new("y", "f32", 4),
                    LayoutField::new("z", "f32", 4),
                ])
            }
        }

        convert_from_into!(
            CustomPoint,
            [
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
                }
            ]
        );
    }

    #[test]
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

        #[derive(Debug, PartialEq, Clone, Default, Copy)]
        #[repr(C, align(4))]
        struct CustomPointXYZI {
            x: f32,
            y: f32,
            z: f32,
            i: u8,
        }

        impl From<IPoint<4>> for CustomPointXYZI {
            fn from(point: IPoint<4>) -> Self {
                Self {
                    x: point[0].get(),
                    y: point[1].get(),
                    z: point[2].get(),
                    i: point[3].get(),
                }
            }
        }

        impl From<CustomPointXYZI> for IPoint<4> {
            fn from(point: CustomPointXYZI) -> Self {
                [
                    point.x.into(),
                    point.y.into(),
                    point.z.into(),
                    point.i.into(),
                ]
                .into()
            }
        }

        unsafe impl PointConvertible<4> for CustomPointXYZI {
            fn layout() -> LayoutDescription {
                LayoutDescription::new(&[
                    LayoutField::new("x", "f32", 4),
                    LayoutField::new("y", "f32", 4),
                    LayoutField::new("z", "f32", 4),
                    LayoutField::new("i", "u8", 1),
                    LayoutField::padding(3),
                ])
            }
        }

        convert_from_into!(CustomPointXYZI, cloud);
    }

    #[test]
    #[cfg(feature = "derive")]
    fn custom_rgba_f32() {
        #[derive(Debug, PartialEq, Clone, Default, Copy)]
        #[repr(C, align(4))]
        struct CustomPoint {
            x: f32,
            y: f32,
            z: f32,
            r: u8,
            g: u8,
            b: u8,
            a: u8,
        }

        impl From<IPoint<7>> for CustomPoint {
            fn from(point: IPoint<7>) -> Self {
                Self {
                    x: point[0].get(),
                    y: point[1].get(),
                    z: point[2].get(),
                    r: point[3].get(),
                    g: point[4].get(),
                    b: point[5].get(),
                    a: point[6].get(),
                }
            }
        }

        impl From<CustomPoint> for IPoint<7> {
            fn from(point: CustomPoint) -> Self {
                [
                    point.x.into(),
                    point.y.into(),
                    point.z.into(),
                    point.r.into(),
                    point.g.into(),
                    point.b.into(),
                    point.a.into(),
                ]
                .into()
            }
        }

        unsafe impl PointConvertible<7> for CustomPoint {
            fn layout() -> LayoutDescription {
                LayoutDescription::new(&[
                    LayoutField::new("x", "f32", 4),
                    LayoutField::new("y", "f32", 4),
                    LayoutField::new("z", "f32", 4),
                    LayoutField::new("r", "u8", 1),
                    LayoutField::padding(3),
                    LayoutField::new("g", "u8", 1),
                    LayoutField::padding(3),
                    LayoutField::new("b", "u8", 1),
                    LayoutField::padding(3),
                    LayoutField::new("a", "u8", 1),
                    LayoutField::padding(3),
                ])
            }
        }

        let cloud = [
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
        let cloud = [
            PointXYZ::new(0.0, 1.0, 5.0),
            PointXYZ::new(1.0, 1.5, 5.0),
            PointXYZ::new(1.3, 1.6, 5.7),
            PointXYZ::new(f32::MAX, f32::MIN, f32::MAX),
        ];

        convert_from_into!(PointXYZ, cloud);
    }

    #[test]
    fn converterxyzrgba() {
        convert_from_into!(
            PointXYZRGBA,
            [
                PointXYZRGBA::new(0.0, 1.0, 5.0, 0, 0, 0, 0),
                PointXYZRGBA::new(1.0, 1.5, 5.0, 1, 1, 1, 1),
                PointXYZRGBA::new(1.3, 1.6, 5.7, 2, 2, 2, 2),
                PointXYZRGBA::new(
                    f32::MAX,
                    f32::MIN,
                    f32::MAX,
                    u8::MAX,
                    u8::MAX,
                    u8::MAX,
                    u8::MAX
                )
            ]
        );
    }

    #[test]
    fn converterxyzinormal() {
        convert_from_into!(
            PointXYZINormal,
            [
                PointXYZINormal::new(0.0, 1.0, 5.0, 0.0, 0.0, 0.0, 0.0),
                PointXYZINormal::new(1.0, 1.5, 5.0, 1.0, 1.0, 1.0, 1.0),
                PointXYZINormal::new(1.3, 1.6, 5.7, 2.0, 2.0, 2.0, 2.0)
            ]
        );
    }

    #[test]
    fn converterxyzrgbnormal() {
        convert_from_into!(
            PointXYZRGBNormal,
            [
                PointXYZRGBNormal::new(0.0, 1.0, 5.0, RGB::new(0, 0, 0), 0.0, 0.0, 0.0),
                PointXYZRGBNormal::new(1.0, 1.5, 5.0, RGB::new(1, 1, 1), 1.0, 1.0, 1.0),
                PointXYZRGBNormal::new(1.3, 1.6, 5.7, RGB::new(2, 2, 2), 2.0, 2.0, 2.0),
                PointXYZRGBNormal::new(
                    f32::MAX,
                    f32::MIN,
                    f32::MAX,
                    RGB::new(u8::MAX, u8::MAX, u8::MAX),
                    f32::MAX,
                    f32::MAX,
                    f32::MAX,
                )
            ]
        );
    }

    #[test]
    fn converterxyznormal() {
        convert_from_into!(
            PointXYZNormal,
            [
                PointXYZNormal::new(0.0, 1.0, 5.0, 0.0, 0.0, 0.0),
                PointXYZNormal::new(1.0, 1.5, 5.0, 1.0, 1.0, 1.0),
                PointXYZNormal::new(1.3, 1.6, 5.7, 2.0, 2.0, 2.0),
                PointXYZNormal::new(f32::MAX, f32::MIN, f32::MAX, f32::MAX, f32::MAX, f32::MAX)
            ]
        );
    }

    #[test]
    fn converterxyzrgbl() {
        convert_from_into!(
            PointXYZRGBL,
            [
                PointXYZRGBL::new(0.0, 1.0, 5.0, 0, 0, 0, 0),
                PointXYZRGBL::new(1.0, 1.5, 5.0, 1, 1, 1, 1),
                PointXYZRGBL::new(1.3, 1.6, 5.7, 2, 2, 2, 2),
                PointXYZRGBL::new(
                    f32::MAX,
                    f32::MIN,
                    f32::MAX,
                    u8::MAX,
                    u8::MAX,
                    u8::MAX,
                    u32::MAX
                )
            ]
        );
    }

    #[test]
    fn converterxyzrgb() {
        convert_from_into!(
            PointXYZRGB,
            [
                PointXYZRGB::new(0.0, 1.0, 5.0, 0, 0, 0),
                PointXYZRGB::new(1.0, 1.5, 5.0, 1, 1, 1),
                PointXYZRGB::new(1.3, 1.6, 5.7, 2, 2, 2),
                PointXYZRGB::new(f32::MAX, f32::MIN, f32::MAX, u8::MAX, u8::MAX, u8::MAX)
            ]
        );
    }

    #[test]
    fn converterxyzrgb_from_vec() {
        convert_from_into_vec!(
            PointXYZRGB,
            vec![
                PointXYZRGB::new(0.0, 1.0, 5.0, 0, 0, 0),
                PointXYZRGB::new(1.3, 1.6, 5.7, 2, 2, 2),
                PointXYZRGB::new(f32::MAX, f32::MIN, f32::MAX, u8::MAX, u8::MAX, u8::MAX),
            ]
        );
    }

    #[test]
    fn converterxyzl() {
        convert_from_into!(
            PointXYZL,
            [
                PointXYZL::new(0.0, 1.0, 5.0, 0),
                PointXYZL::new(1.0, 1.5, 5.0, 1),
                PointXYZL::new(1.3, 1.6, 5.7, 2),
                PointXYZL::new(f32::MAX, f32::MIN, f32::MAX, u32::MAX)
            ]
        );
    }

    #[test]
    fn converterxyzi() {
        convert_from_into!(
            PointXYZI,
            [
                PointXYZI::new(0.0, 1.0, 5.0, 0.0),
                PointXYZI::new(1.0, 1.5, 5.0, 1.0),
                PointXYZI::new(1.3, 1.6, 5.7, 2.0),
                PointXYZI::new(f32::MAX, f32::MIN, f32::MAX, f32::MAX)
            ]
        );
    }

    #[test]
    fn write_xyzi_read_xyz() {
        let write_cloud = [
            PointXYZI::new(0.0, 1.0, 5.0, 0.0),
            PointXYZI::new(1.0, 1.5, 5.0, 1.0),
            PointXYZI::new(1.3, 1.6, 5.7, 2.0),
            PointXYZI::new(f32::MAX, f32::MIN, f32::MAX, f32::MAX),
        ];

        let read_cloud = [
            PointXYZ::new(0.0, 1.0, 5.0),
            PointXYZ::new(1.0, 1.5, 5.0),
            PointXYZ::new(1.3, 1.6, 5.7),
            PointXYZ::new(f32::MAX, f32::MIN, f32::MAX),
        ];

        convert_from_into_in_out_cloud!(write_cloud, PointXYZI, read_cloud, PointXYZ);
    }

    #[test]
    fn write_xyzi_read_xyz_vec() {
        let write_cloud = vec![
            PointXYZI::new(0.0, 1.0, 5.0, 0.0),
            PointXYZI::new(1.0, 1.5, 5.0, 1.0),
            PointXYZI::new(1.3, 1.6, 5.7, 2.0),
            PointXYZI::new(f32::MAX, f32::MIN, f32::MAX, f32::MAX),
        ];

        let read_cloud = [
            PointXYZ::new(0.0, 1.0, 5.0),
            PointXYZ::new(1.0, 1.5, 5.0),
            PointXYZ::new(1.3, 1.6, 5.7),
            PointXYZ::new(f32::MAX, f32::MIN, f32::MAX),
        ];

        convert_from_into_in_out_cloud_vec!(write_cloud, PointXYZI, read_cloud, PointXYZ);
    }

    #[test]
    fn write_less_than_available() {
        #[derive(Debug, PartialEq, Clone, Default, Copy)]
        #[repr(C, align(4))]
        struct CustomPoint {
            x: f32,
            y: f32,
            z: f32,
            dummy: f32,
        }

        impl From<IPoint<3>> for CustomPoint {
            fn from(point: IPoint<3>) -> Self {
                Self {
                    x: point[0].get(),
                    y: point[1].get(),
                    z: point[2].get(),
                    dummy: 0.0,
                }
            }
        }

        impl From<CustomPoint> for IPoint<3> {
            fn from(point: CustomPoint) -> Self {
                [point.x.into(), point.y.into(), point.z.into()].into()
            }
        }

        unsafe impl PointConvertible<3> for CustomPoint {
            fn layout() -> LayoutDescription {
                LayoutDescription::new(&[
                    LayoutField::new("x", "f32", 4),
                    LayoutField::new("y", "f32", 4),
                    LayoutField::new("z", "f32", 4),
                ])
            }
        }

        let write_cloud = [
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
}
