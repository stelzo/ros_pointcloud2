use crate::{MetaNames, Point, PointConvertible};

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
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct PointXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl From<Point<f32, 3, 0>> for PointXYZ {
    fn from(point: Point<f32, 3, 0>) -> Self {
        Self {
            x: point.coords[0],
            y: point.coords[1],
            z: point.coords[2],
        }
    }
}

impl From<PointXYZ> for Point<f32, 3, 0> {
    fn from(point: PointXYZ) -> Self {
        Point {
            coords: [point.x, point.y, point.z],
            meta: [],
        }
    }
}

impl MetaNames<0> for PointXYZ {
    fn meta_names() -> [&'static str; 0] {
        []
    }
}

impl PointConvertible<f32, { std::mem::size_of::<f32>() }, 3, 0> for PointXYZ {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and an intensity value.
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

impl From<PointXYZI> for Point<f32, 3, 1> {
    fn from(point: PointXYZI) -> Self {
        Point {
            coords: [point.x, point.y, point.z],
            meta: [point.intensity.into()],
        }
    }
}

impl From<Point<f32, 3, 1>> for PointXYZI {
    fn from(point: Point<f32, 3, 1>) -> Self {
        Self {
            x: point.coords[0],
            y: point.coords[1],
            z: point.coords[2],
            intensity: point.meta[0].get(),
        }
    }
}

impl MetaNames<1> for PointXYZI {
    fn meta_names() -> [&'static str; 1] {
        ["intensity"]
    }
}

impl PointConvertible<f32, { std::mem::size_of::<f32>() }, 3, 1> for PointXYZI {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and an RGB color value.
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct PointXYZRGB {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl From<Point<f32, 3, 1>> for PointXYZRGB {
    fn from(point: Point<f32, 3, 1>) -> Self {
        let rgb = point.meta[0].get::<f32>();
        let rgb_unpacked = unpack_rgb(rgb);
        Self {
            x: point.coords[0],
            y: point.coords[1],
            z: point.coords[2],
            r: rgb_unpacked[0],
            g: rgb_unpacked[1],
            b: rgb_unpacked[2],
        }
    }
}

impl From<PointXYZRGB> for Point<f32, 3, 1> {
    fn from(point: PointXYZRGB) -> Self {
        Point {
            coords: [point.x, point.y, point.z],
            meta: [pack_rgb(point.r, point.g, point.b).into()],
        }
    }
}

impl MetaNames<1> for PointXYZRGB {
    fn meta_names() -> [&'static str; 1] {
        ["rgb"]
    }
}

impl PointConvertible<f32, { std::mem::size_of::<f32>() }, 3, 1> for PointXYZRGB {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and an RGBA color value.
/// The alpha channel is commonly used as padding but this crate uses every channel and no padding.
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct PointXYZRGBA {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl From<Point<f32, 3, 2>> for PointXYZRGBA {
    fn from(point: Point<f32, 3, 2>) -> Self {
        let rgb = point.meta[0].get::<f32>();
        let rgb_unpacked = unpack_rgb(rgb);
        Self {
            x: point.coords[0],
            y: point.coords[1],
            z: point.coords[2],
            r: rgb_unpacked[0],
            g: rgb_unpacked[1],
            b: rgb_unpacked[2],
            a: point.meta[1].get(),
        }
    }
}

impl From<PointXYZRGBA> for Point<f32, 3, 2> {
    fn from(point: PointXYZRGBA) -> Self {
        let rgb = pack_rgb(point.r, point.g, point.b);
        Point {
            coords: [point.x, point.y, point.z],
            meta: [rgb.into(), point.a.into()],
        }
    }
}

impl MetaNames<2> for PointXYZRGBA {
    fn meta_names() -> [&'static str; 2] {
        ["rgb", "a"]
    }
}

impl PointConvertible<f32, { std::mem::size_of::<f32>() }, 3, 2> for PointXYZRGBA {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates, an RGB color value and a normal vector.
#[derive(Clone, Debug, PartialEq, Copy)]
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

impl From<Point<f32, 3, 4>> for PointXYZRGBNormal {
    fn from(point: Point<f32, 3, 4>) -> Self {
        let rgb = point.meta[0].get::<f32>();
        let rgb_unpacked = unpack_rgb(rgb);
        Self {
            x: point.coords[0],
            y: point.coords[1],
            z: point.coords[2],
            r: rgb_unpacked[0],
            g: rgb_unpacked[1],
            b: rgb_unpacked[2],
            normal_x: point.meta[1].get(),
            normal_y: point.meta[2].get(),
            normal_z: point.meta[3].get(),
        }
    }
}

impl From<PointXYZRGBNormal> for Point<f32, 3, 4> {
    fn from(point: PointXYZRGBNormal) -> Self {
        let rgb = pack_rgb(point.r, point.g, point.b);
        Point {
            coords: [point.x, point.y, point.z],
            meta: [
                rgb.into(),
                point.normal_x.into(),
                point.normal_y.into(),
                point.normal_z.into(),
            ],
        }
    }
}

impl MetaNames<4> for PointXYZRGBNormal {
    fn meta_names() -> [&'static str; 4] {
        ["rgb", "normal_x", "normal_y", "normal_z"]
    }
}

impl PointConvertible<f32, { std::mem::size_of::<f32>() }, 3, 4> for PointXYZRGBNormal {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates, an intensity value and a normal vector.
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct PointXYZINormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
}

impl From<PointXYZINormal> for Point<f32, 3, 4> {
    fn from(point: PointXYZINormal) -> Self {
        Point {
            coords: [point.x, point.y, point.z],
            meta: [
                point.intensity.into(),
                point.normal_x.into(),
                point.normal_y.into(),
                point.normal_z.into(),
            ],
        }
    }
}

impl From<Point<f32, 3, 4>> for PointXYZINormal {
    fn from(point: Point<f32, 3, 4>) -> Self {
        Self {
            x: point.coords[0],
            y: point.coords[1],
            z: point.coords[2],
            intensity: point.meta[0].get(),
            normal_x: point.meta[1].get(),
            normal_y: point.meta[2].get(),
            normal_z: point.meta[3].get(),
        }
    }
}

impl MetaNames<4> for PointXYZINormal {
    fn meta_names() -> [&'static str; 4] {
        ["intensity", "normal_x", "normal_y", "normal_z"]
    }
}

impl PointConvertible<f32, { std::mem::size_of::<f32>() }, 3, 4> for PointXYZINormal {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and a label.
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct PointXYZL {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub label: u32,
}

impl From<Point<f32, 3, 1>> for PointXYZL {
    fn from(point: Point<f32, 3, 1>) -> Self {
        Self {
            x: point.coords[0],
            y: point.coords[1],
            z: point.coords[2],
            label: point.meta[0].get(),
        }
    }
}

impl From<PointXYZL> for Point<f32, 3, 1> {
    fn from(point: PointXYZL) -> Self {
        Point {
            coords: [point.x, point.y, point.z],
            meta: [point.label.into()],
        }
    }
}

impl MetaNames<1> for PointXYZL {
    fn meta_names() -> [&'static str; 1] {
        ["label"]
    }
}

impl PointConvertible<f32, { std::mem::size_of::<f32>() }, 3, 1> for PointXYZL {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and a label.
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct PointXYZRGBL {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub label: u32,
}

impl From<Point<f32, 3, 2>> for PointXYZRGBL {
    fn from(point: Point<f32, 3, 2>) -> Self {
        let rgb = point.meta[0].get::<f32>();
        let rgb_unpacked = unpack_rgb(rgb);
        Self {
            x: point.coords[0],
            y: point.coords[1],
            z: point.coords[2],
            r: rgb_unpacked[0],
            g: rgb_unpacked[1],
            b: rgb_unpacked[2],
            label: point.meta[1].get(),
        }
    }
}

impl From<PointXYZRGBL> for Point<f32, 3, 2> {
    fn from(point: PointXYZRGBL) -> Self {
        Point {
            coords: [point.x, point.y, point.z],
            meta: [
                pack_rgb(point.r, point.g, point.b).into(),
                point.label.into(),
            ],
        }
    }
}

impl MetaNames<2> for PointXYZRGBL {
    fn meta_names() -> [&'static str; 2] {
        ["rgb", "label"]
    }
}

impl PointConvertible<f32, { std::mem::size_of::<f32>() }, 3, 2> for PointXYZRGBL {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and a normal vector.
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct PointXYZNormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
}

impl From<Point<f32, 3, 3>> for PointXYZNormal {
    fn from(point: Point<f32, 3, 3>) -> Self {
        Self {
            x: point.coords[0],
            y: point.coords[1],
            z: point.coords[2],
            normal_x: point.meta[0].get(),
            normal_y: point.meta[1].get(),
            normal_z: point.meta[2].get(),
        }
    }
}

impl From<PointXYZNormal> for Point<f32, 3, 3> {
    fn from(point: PointXYZNormal) -> Self {
        Point {
            coords: [point.x, point.y, point.z],
            meta: [
                point.normal_x.into(),
                point.normal_y.into(),
                point.normal_z.into(),
            ],
        }
    }
}

impl MetaNames<3> for PointXYZNormal {
    fn meta_names() -> [&'static str; 3] {
        ["normal_x", "normal_y", "normal_z"]
    }
}

impl PointConvertible<f32, { std::mem::size_of::<f32>() }, 3, 3> for PointXYZNormal {}
