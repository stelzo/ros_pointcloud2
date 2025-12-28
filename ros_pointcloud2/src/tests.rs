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
}
