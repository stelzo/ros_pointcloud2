use crate::{ConversionError, MetaNames, PointConvertible, PointMeta};

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates.
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct PointXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl TryInto<([f32; 3], [PointMeta; 0])> for PointXYZ {
    type Error = ConversionError;

    fn try_into(self) -> Result<([f32; 3], [PointMeta; 0]), Self::Error> {
        Ok(([self.x, self.y, self.z], []))
    }
}

impl TryFrom<([f32; 3], [PointMeta; 0])> for PointXYZ {
    type Error = ConversionError;

    fn try_from(data: ([f32; 3], [PointMeta; 0])) -> Result<Self, Self::Error> {
        Ok(Self {
            x: data.0[0],
            y: data.0[1],
            z: data.0[2],
        })
    }
}

impl MetaNames<0> for PointXYZ {
    fn meta_names() -> [String; 0] {
        []
    }
}

impl PointConvertible<f32, {size_of!(f32)}, 3, 0> for PointXYZ {}


/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and an intensity value.
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

impl TryInto<([f32; 3], [PointMeta; 1])> for PointXYZI {
    type Error = ConversionError;

    fn try_into(self) -> Result<([f32; 3], [PointMeta; 1]), Self::Error> {
        Ok(([self.x, self.y, self.z], [PointMeta::new(self.intensity)]))
    }
}

impl TryFrom<([f32; 3], [PointMeta; 1])> for PointXYZI {
    type Error = ConversionError;

    fn try_from(data: ([f32; 3], [PointMeta; 1])) -> Result<Self, Self::Error> {
        Ok(Self {
            x: data.0[0],
            y: data.0[1],
            z: data.0[2],
            intensity: data.1[0].get().unwrap(),
        })
    }
}

impl MetaNames<1> for PointXYZI {
    fn meta_names() -> [String; 1] {
        ["intensity"].map(|s| s.to_string())
    }
}

impl PointConvertible<f32, {size_of!(f32)}, 3, 1> for PointXYZI {}

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

impl TryInto<([f32; 3], [PointMeta; 3])> for PointXYZRGB {
    type Error = ConversionError;

    fn try_into(self) -> Result<([f32; 3], [PointMeta; 3]), Self::Error> {
        Ok(([self.x, self.y, self.z], [
            PointMeta::new(self.r),
            PointMeta::new(self.g),
            PointMeta::new(self.b),
        ]))
    }
}

impl TryFrom<([f32; 3], [PointMeta; 3])> for PointXYZRGB {
    type Error = ConversionError;

    fn try_from(data: ([f32; 3], [PointMeta; 3])) -> Result<Self, Self::Error> {
        Ok(Self {
            x: data.0[0],
            y: data.0[1],
            z: data.0[2],
            r: data.1[0].get().unwrap(),
            g: data.1[1].get().unwrap(),
            b: data.1[2].get().unwrap(),
        })
    }
}

impl MetaNames<3> for PointXYZRGB {
    fn meta_names() -> [String; 3] {
        ["r", "g", "b"].map(|s| s.to_string())
    }
}

impl PointConvertible<f32, {size_of!(f32)}, 3, 3> for PointXYZRGB {}

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

impl TryInto<([f32; 3], [PointMeta; 4])> for PointXYZRGBA {
    type Error = ConversionError;

    fn try_into(self) -> Result<([f32; 3], [PointMeta; 4]), Self::Error> {
        Ok(([self.x, self.y, self.z], [
            PointMeta::new(self.r),
            PointMeta::new(self.g),
            PointMeta::new(self.b),
            PointMeta::new(self.a),
        ]))
    }
}

impl TryFrom<([f32; 3], [PointMeta; 4])> for PointXYZRGBA {
    type Error = ConversionError;

    fn try_from(data: ([f32; 3], [PointMeta; 4])) -> Result<Self, Self::Error> {
        Ok(Self {
            x: data.0[0],
            y: data.0[1],
            z: data.0[2],
            r: data.1[0].get().unwrap(),
            g: data.1[1].get().unwrap(),
            b: data.1[2].get().unwrap(),
            a: data.1[3].get().unwrap(),
        })
    }
}

impl MetaNames<4> for PointXYZRGBA {
    fn meta_names() -> [String; 4] {
        [
            "r",
            "g",
            "b",
            "a",
        ].map(|s| s.to_string())
    }
}

impl PointConvertible<f32, {size_of!(f32)}, 3, 4> for PointXYZRGBA {}

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

impl TryInto<([f32; 3], [PointMeta; 6])> for PointXYZRGBNormal {
    type Error = ConversionError;

    fn try_into(self) -> Result<([f32; 3], [PointMeta; 6]), Self::Error> {
        Ok(([self.x, self.y, self.z], [
            PointMeta::new(self.r),
            PointMeta::new(self.g),
            PointMeta::new(self.b),
            PointMeta::new(self.normal_x),
            PointMeta::new(self.normal_y),
            PointMeta::new(self.normal_z),
        ]))
    }
}

impl TryFrom<([f32; 3], [PointMeta; 6])> for PointXYZRGBNormal {
    type Error = ConversionError;

    fn try_from(data: ([f32; 3], [PointMeta; 6])) -> Result<Self, Self::Error> {
        Ok(Self {
            x: data.0[0],
            y: data.0[1],
            z: data.0[2],
            r: data.1[0].get().unwrap(),
            g: data.1[1].get().unwrap(),
            b: data.1[2].get().unwrap(),
            normal_x: data.1[3].get().unwrap(),
            normal_y: data.1[4].get().unwrap(),
            normal_z: data.1[5].get().unwrap(),
        })
    }
}

impl MetaNames<6> for PointXYZRGBNormal {
    fn meta_names() -> [String; 6] {
        [
            "r",
            "g",
            "b",
            "normal_x",
            "normal_y",
            "normal_z",
        ].map(|s| s.to_string())
    }
}

impl PointConvertible<f32, {size_of!(f32)}, 3, 6> for PointXYZRGBNormal {}

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

impl TryInto<([f32; 3], [PointMeta; 4])> for PointXYZINormal {
    type Error = ConversionError;

    fn try_into(self) -> Result<([f32; 3], [PointMeta; 4]), Self::Error> {
        Ok(([self.x, self.y, self.z], [
            PointMeta::new(self.intensity),
            PointMeta::new(self.normal_x),
            PointMeta::new(self.normal_y),
            PointMeta::new(self.normal_z),
        ]))
    }
}

impl TryFrom<([f32; 3], [PointMeta; 4])> for PointXYZINormal {
    type Error = ConversionError;

    fn try_from(data: ([f32; 3], [PointMeta; 4])) -> Result<Self, Self::Error> {
        Ok(Self {
            x: data.0[0],
            y: data.0[1],
            z: data.0[2],
            intensity: data.1[0].get().unwrap(),
            normal_x: data.1[1].get().unwrap(),
            normal_y: data.1[2].get().unwrap(),
            normal_z: data.1[3].get().unwrap(),
        })
    }
}

impl MetaNames<4> for PointXYZINormal {
    fn meta_names() -> [String; 4] {
        [
            "intensity",
            "normal_x",
            "normal_y",
            "normal_z",
        ].map(|s| s.to_string())
    }
}

impl PointConvertible<f32, {size_of!(f32)}, 3, 4> for PointXYZINormal {}

/// Predefined point type commonly used in ROS with PCL.
/// This is a 3D point with x, y, z coordinates and a label.
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct PointXYZL {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub label: u32,
}

impl TryInto<([f32; 3], [PointMeta; 1])> for PointXYZL {
    type Error = ConversionError;

    fn try_into(self) -> Result<([f32; 3], [PointMeta; 1]), Self::Error> {
        Ok(([self.x, self.y, self.z], [
            PointMeta::new(self.label),
        ]))
    }
}

impl TryFrom<([f32; 3], [PointMeta; 1])> for PointXYZL {
    type Error = ConversionError;

    fn try_from(data: ([f32; 3], [PointMeta; 1])) -> Result<Self, Self::Error> {
        Ok(Self {
            x: data.0[0],
            y: data.0[1],
            z: data.0[2],
            label: data.1[0].get().unwrap(),
        })
    }
}

impl MetaNames<1> for PointXYZL {
    fn meta_names() -> [String; 1] {
        ["label".to_string()]
    }
}

impl PointConvertible<f32, {size_of!(f32)}, 3, 1> for PointXYZL {}

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

impl TryInto<([f32; 3], [PointMeta; 4])> for PointXYZRGBL {
    type Error = ConversionError;

    fn try_into(self) -> Result<([f32; 3], [PointMeta; 4]), Self::Error> {
        Ok(([self.x, self.y, self.z], [
            PointMeta::new(self.r),
            PointMeta::new(self.g),
            PointMeta::new(self.b),
            PointMeta::new(self.label),
        ]))
    }
}

impl TryFrom<([f32; 3], [PointMeta; 4])> for PointXYZRGBL {
    type Error = ConversionError;

    fn try_from(data: ([f32; 3], [PointMeta; 4])) -> Result<Self, Self::Error> {
        Ok(Self {
            x: data.0[0],
            y: data.0[1],
            z: data.0[2],
            r: data.1[0].get().unwrap(),
            g: data.1[1].get().unwrap(),
            b: data.1[2].get().unwrap(),
            label: data.1[3].get().unwrap(),
        })
    }
}

impl MetaNames<4> for PointXYZRGBL {
    fn meta_names() -> [String; 4] {
        [
            "r",
            "g",
            "b",
            "label",
        ].map(|s| s.to_string())
    }
}

impl PointConvertible<f32, {size_of!(f32)}, 3, 4> for PointXYZRGBL {}

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

impl TryInto<([f32; 3], [PointMeta; 3])> for PointXYZNormal {
    type Error = ConversionError;

    fn try_into(self) -> Result<([f32; 3], [PointMeta; 3]), Self::Error> {
        Ok(([self.x, self.y, self.z], [
            PointMeta::new(self.normal_x),
            PointMeta::new(self.normal_y),
            PointMeta::new(self.normal_z),
        ]))
    }
}

impl TryFrom<([f32; 3], [PointMeta; 3])> for PointXYZNormal {
    type Error = ConversionError;

    fn try_from(data: ([f32; 3], [PointMeta; 3])) -> Result<Self, Self::Error> {
        Ok(Self {
            x: data.0[0],
            y: data.0[1],
            z: data.0[2],
            normal_x: data.1[0].get().unwrap(),
            normal_y: data.1[1].get().unwrap(),
            normal_z: data.1[2].get().unwrap(),
        })
    }
}

impl MetaNames<3> for PointXYZNormal {
    fn meta_names() -> [String; 3] {
        [
            "normal_x",
            "normal_y",
            "normal_z",
        ].map(|s| s.to_string())
    }
}

impl PointConvertible<f32, {size_of!(f32)}, 3, 3> for PointXYZNormal {}
