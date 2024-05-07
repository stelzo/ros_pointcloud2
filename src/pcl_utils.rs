use crate::{ConversionError, Fields, Point, PointConvertible};

#[cfg(feature = "derive")]
use crate::TypeLayout;

/// Pack an RGB color into a single f32 value as used in ROS with PCL for RViz usage.
#[inline]
fn pack_rgb(r: u8, g: u8, b: u8) -> f32 {
    let packed = ((r as u32) << 16) + ((g as u32) << 8) + (b as u32);
    f32::from_bits(packed)
}

/// Unpack an RGB color from a single f32 value as used in ROS with PCL for RViz usage.
#[inline]
fn unpack_rgb(rgb: f32) -> [u8; 3] {
    let packed: u32 = rgb.to_bits();
    let r: u8 = (packed >> 16) as u8;
    let g: u8 = (packed >> 8) as u8;
    let b: u8 = packed as u8;
    [r, g, b]
}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[cfg_attr(feature = "derive", derive(TypeLayout))]
#[repr(C)]
pub struct PointXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Fields<3> for PointXYZ {
    fn field_names_ordered() -> [&'static str; 3] {
        ["x", "y", "z"]
    }
}

impl From<Point<3>> for PointXYZ {
    fn from(point: Point<3>) -> Self {
        Self {
            x: point.fields[0].get(),
            y: point.fields[1].get(),
            z: point.fields[2].get(),
        }
    }
}

impl From<PointXYZ> for Point<3> {
    fn from(point: PointXYZ) -> Self {
        Point {
            fields: [point.x.into(), point.y.into(), point.z.into()],
        }
    }
}

impl PointConvertible<3> for PointXYZ {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and an intensity value.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[cfg_attr(feature = "derive", derive(TypeLayout))]
#[repr(C)]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

impl Fields<4> for PointXYZI {
    fn field_names_ordered() -> [&'static str; 4] {
        ["x", "y", "z", "intensity"]
    }
}

impl From<Point<4>> for PointXYZI {
    fn from(point: Point<4>) -> Self {
        Self {
            x: point.fields[0].get(),
            y: point.fields[1].get(),
            z: point.fields[2].get(),
            intensity: point.fields[3].get(),
        }
    }
}

impl From<PointXYZI> for Point<4> {
    fn from(point: PointXYZI) -> Self {
        Point {
            fields: [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                point.intensity.into(),
            ],
        }
    }
}

impl PointConvertible<4> for PointXYZI {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and a label.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[cfg_attr(feature = "derive", derive(TypeLayout))]
#[repr(C)]
pub struct PointXYZL {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub label: u32,
}

impl Fields<4> for PointXYZL {
    fn field_names_ordered() -> [&'static str; 4] {
        ["x", "y", "z", "label"]
    }
}

impl From<Point<4>> for PointXYZL {
    fn from(point: Point<4>) -> Self {
        Self {
            x: point.fields[0].get(),
            y: point.fields[1].get(),
            z: point.fields[2].get(),
            label: point.fields[3].get(),
        }
    }
}

impl From<PointXYZL> for Point<4> {
    fn from(point: PointXYZL) -> Self {
        Point {
            fields: [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                point.label.into(),
            ],
        }
    }
}

impl PointConvertible<4> for PointXYZL {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and an RGB color value.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[cfg_attr(feature = "derive", derive(TypeLayout))]
#[repr(C)]
pub struct PointXYZRGB {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Fields<4> for PointXYZRGB {
    fn field_names_ordered() -> [&'static str; 4] {
        ["x", "y", "z", "rgb"]
    }
}

impl From<Point<4>> for PointXYZRGB {
    fn from(point: Point<4>) -> Self {
        let rgb = point.fields[3].get::<f32>();
        let rgb_unpacked = unpack_rgb(rgb);
        Self {
            x: point.fields[0].get(),
            y: point.fields[1].get(),
            z: point.fields[2].get(),
            r: rgb_unpacked[0],
            g: rgb_unpacked[1],
            b: rgb_unpacked[2],
        }
    }
}

impl From<PointXYZRGB> for Point<4> {
    fn from(point: PointXYZRGB) -> Self {
        let rgb = pack_rgb(point.r, point.g, point.b);
        Point {
            fields: [point.x.into(), point.y.into(), point.z.into(), rgb.into()],
        }
    }
}

impl PointConvertible<4> for PointXYZRGB {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and an RGBA color value.
/// The alpha channel is commonly used as padding but this crate uses every channel and no padding.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[cfg_attr(feature = "derive", derive(TypeLayout))]
#[repr(C)]
pub struct PointXYZRGBA {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl Fields<5> for PointXYZRGBA {
    fn field_names_ordered() -> [&'static str; 5] {
        ["x", "y", "z", "rgb", "a"]
    }
}

impl From<Point<5>> for PointXYZRGBA {
    fn from(point: Point<5>) -> Self {
        let rgb = point.fields[3].get::<f32>();
        let rgb_unpacked = unpack_rgb(rgb);
        Self {
            x: point.fields[0].get(),
            y: point.fields[1].get(),
            z: point.fields[2].get(),
            r: rgb_unpacked[0],
            g: rgb_unpacked[1],
            b: rgb_unpacked[2],
            a: point.fields[4].get(),
        }
    }
}

impl From<PointXYZRGBA> for Point<5> {
    fn from(point: PointXYZRGBA) -> Self {
        let rgb = pack_rgb(point.r, point.g, point.b);
        Point {
            fields: [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                rgb.into(),
                point.a.into(),
            ],
        }
    }
}

impl PointConvertible<5> for PointXYZRGBA {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates, an RGB color value and a normal vector.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[cfg_attr(feature = "derive", derive(TypeLayout))]
#[repr(C)]
pub struct PointXYZRGBNormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
}

impl Fields<7> for PointXYZRGBNormal {
    fn field_names_ordered() -> [&'static str; 7] {
        ["x", "y", "z", "rgb", "normal_x", "normal_y", "normal_z"]
    }
}

impl From<Point<7>> for PointXYZRGBNormal {
    fn from(point: Point<7>) -> Self {
        let rgb = point.fields[3].get::<f32>();
        let rgb_unpacked = unpack_rgb(rgb);
        Self {
            x: point.fields[0].get(),
            y: point.fields[1].get(),
            z: point.fields[2].get(),
            r: rgb_unpacked[0],
            g: rgb_unpacked[1],
            b: rgb_unpacked[2],
            normal_x: point.fields[4].get(),
            normal_y: point.fields[5].get(),
            normal_z: point.fields[6].get(),
        }
    }
}

impl From<PointXYZRGBNormal> for Point<7> {
    fn from(point: PointXYZRGBNormal) -> Self {
        let rgb = pack_rgb(point.r, point.g, point.b);
        Point {
            fields: [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                rgb.into(),
                point.normal_x.into(),
                point.normal_y.into(),
                point.normal_z.into(),
            ],
        }
    }
}

impl PointConvertible<7> for PointXYZRGBNormal {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates, an intensity value and a normal vector.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[cfg_attr(feature = "derive", derive(TypeLayout))]
#[repr(C)]
pub struct PointXYZINormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
}

impl Fields<7> for PointXYZINormal {
    fn field_names_ordered() -> [&'static str; 7] {
        ["x", "y", "z", "i", "normal_x", "normal_y", "normal_z"]
    }
}

impl From<Point<7>> for PointXYZINormal {
    fn from(point: Point<7>) -> Self {
        Self {
            x: point.fields[0].get(),
            y: point.fields[1].get(),
            z: point.fields[2].get(),
            intensity: point.fields[3].get(),
            normal_x: point.fields[4].get(),
            normal_y: point.fields[5].get(),
            normal_z: point.fields[6].get(),
        }
    }
}

impl From<PointXYZINormal> for Point<7> {
    fn from(point: PointXYZINormal) -> Self {
        Point {
            fields: [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                point.intensity.into(),
                point.normal_x.into(),
                point.normal_y.into(),
                point.normal_z.into(),
            ],
        }
    }
}

impl PointConvertible<7> for PointXYZINormal {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and a label.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[cfg_attr(feature = "derive", derive(TypeLayout))]
#[repr(C)]
pub struct PointXYZRGBL {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub label: u32,
}

impl Fields<5> for PointXYZRGBL {
    fn field_names_ordered() -> [&'static str; 5] {
        ["x", "y", "z", "rgb", "label"]
    }
}

impl From<Point<5>> for PointXYZRGBL {
    fn from(point: Point<5>) -> Self {
        let rgb = point.fields[3].get::<f32>();
        let rgb_unpacked = unpack_rgb(rgb);
        Self {
            x: point.fields[0].get(),
            y: point.fields[1].get(),
            z: point.fields[2].get(),
            r: rgb_unpacked[0],
            g: rgb_unpacked[1],
            b: rgb_unpacked[2],
            label: point.fields[4].get(),
        }
    }
}

impl From<PointXYZRGBL> for Point<5> {
    fn from(point: PointXYZRGBL) -> Self {
        let rgb = pack_rgb(point.r, point.g, point.b);
        Point {
            fields: [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                rgb.into(),
                point.label.into(),
            ],
        }
    }
}

impl PointConvertible<5> for PointXYZRGBL {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and a normal vector.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[cfg_attr(feature = "derive", derive(TypeLayout))]
#[repr(C)]
pub struct PointXYZNormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
}

impl Fields<6> for PointXYZNormal {
    fn field_names_ordered() -> [&'static str; 6] {
        ["x", "y", "z", "normal_x", "normal_y", "normal_z"]
    }
}

impl From<Point<6>> for PointXYZNormal {
    fn from(point: Point<6>) -> Self {
        Self {
            x: point.fields[0].get(),
            y: point.fields[1].get(),
            z: point.fields[2].get(),
            normal_x: point.fields[3].get(),
            normal_y: point.fields[4].get(),
            normal_z: point.fields[5].get(),
        }
    }
}

impl From<PointXYZNormal> for Point<6> {
    fn from(point: PointXYZNormal) -> Self {
        Point {
            fields: [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                point.normal_x.into(),
                point.normal_y.into(),
                point.normal_z.into(),
            ],
        }
    }
}

impl PointConvertible<6> for PointXYZNormal {}
