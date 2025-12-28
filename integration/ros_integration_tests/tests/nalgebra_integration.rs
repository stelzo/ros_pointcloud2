#![cfg(feature = "nalgebra")]

// Invoke the consumer-side macro that generates the nalgebra helpers for PointXYZ
ros_pointcloud2::impl_pointxyz_for_nalgebra!();

use impl_nalgebra::AsNalgebra;
use nalgebra::Point3;
use ros_pointcloud2::prelude::{PointXYZ, PointXYZI, PointXYZL, PointXYZRGB, PointXYZRGBA, PointXYZRGBNormal, PointXYZINormal, PointXYZRGBL, PointXYZNormal, RGB};

#[test]
fn impl_pointxyz_for_nalgebra_all_point_types_xyz() {
    // PointXYZ
    let p_xyz = PointXYZ::new(1.0, 2.0, 3.0);
    assert_eq!(AsNalgebra::xyz(&p_xyz), Point3::new(1.0, 2.0, 3.0));

    // PointXYZI
    let p_xyzi = PointXYZI::new(4.0, 5.0, 6.0, 7.0);
    assert_eq!(AsNalgebra::xyz(&p_xyzi), Point3::new(4.0, 5.0, 6.0));

    // PointXYZL
    let p_xyzl = PointXYZL::new(8.0, 9.0, 10.0, 123);
    assert_eq!(AsNalgebra::xyz(&p_xyzl), Point3::new(8.0, 9.0, 10.0));

    // PointXYZRGB
    let p_xyzrgb = PointXYZRGB::new(11.0, 12.0, 13.0, 1, 2, 3);
    assert_eq!(AsNalgebra::xyz(&p_xyzrgb), Point3::new(11.0, 12.0, 13.0));

    // PointXYZRGBA
    let p_xyzrgba = PointXYZRGBA::new(14.0, 15.0, 16.0, 4, 5, 6, 7);
    assert_eq!(AsNalgebra::xyz(&p_xyzrgba), Point3::new(14.0, 15.0, 16.0));

    // PointXYZRGBNormal
    let p_xyzrgbn = PointXYZRGBNormal::new(17.0, 18.0, 19.0, RGB::new(1,2,3), 0.1, 0.2, 0.3);
    assert_eq!(AsNalgebra::xyz(&p_xyzrgbn), Point3::new(17.0, 18.0, 19.0));

    // PointXYZINormal
    let p_xyzin = PointXYZINormal::new(20.0, 21.0, 22.0, 8.0, 0.1, 0.2, 0.3);
    assert_eq!(AsNalgebra::xyz(&p_xyzin), Point3::new(20.0, 21.0, 22.0));

    // PointXYZRGBL
    let p_xyzrgbl = PointXYZRGBL::new(23.0, 24.0, 25.0, 7, 8, 9, 42);
    assert_eq!(AsNalgebra::xyz(&p_xyzrgbl), Point3::new(23.0, 24.0, 25.0));

    // PointXYZNormal
    let p_xyzn = PointXYZNormal::new(26.0, 27.0, 28.0, 0.1, 0.2, 0.3);
    assert_eq!(AsNalgebra::xyz(&p_xyzn), Point3::new(26.0, 27.0, 28.0));
}
