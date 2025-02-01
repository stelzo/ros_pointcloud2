//! Predefined point types commonly used in ROS.
use crate::{LayoutDescription, LayoutField, PointConvertible, RPCL2Point};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Packed RGB color encoding as used in ROS tools.
#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub union RGB {
    packed: f32,
    unpacked: [u8; 4], // Padding
}

#[cfg(feature = "serde")]
#[cfg_attr(docsrs, doc(cfg(feature = "serde")))]
impl<'de> Deserialize<'de> for RGB {
    fn deserialize<D>(deserializer: D) -> Result<RGB, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let packed = f32::deserialize(deserializer)?;
        Ok(RGB::new_from_packed_f32(packed))
    }
}

#[cfg(feature = "serde")]
#[cfg_attr(docsrs, doc(cfg(feature = "serde")))]
impl Serialize for RGB {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        f32::from(*self).serialize(serializer)
    }
}

unsafe impl Send for RGB {}
unsafe impl Sync for RGB {}

impl Default for RGB {
    fn default() -> Self {
        Self { packed: 0.0 }
    }
}

impl PartialEq for RGB {
    fn eq(&self, other: &Self) -> bool {
        self.r() == other.r() && self.g() == other.g() && self.b() == other.b()
    }
}

impl core::fmt::Display for RGB {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "#{:02X}{:02X}{:02X}", self.r(), self.g(), self.b())
    }
}

impl core::fmt::Debug for RGB {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("RGB")
            .field("r", &self.r())
            .field("g", &self.g())
            .field("b", &self.b())
            .finish()
    }
}

impl RGB {
    #[must_use]
    pub fn new(r: u8, g: u8, b: u8) -> Self {
        Self {
            unpacked: [b, g, r, 0],
        }
    }

    #[must_use]
    pub fn new_from_packed_f32(packed: f32) -> Self {
        Self { packed }
    }

    #[must_use]
    pub fn new_from_packed(packed: u32) -> Self {
        Self::new_from_packed_f32(f32::from_bits(packed))
    }

    #[must_use]
    pub fn raw(&self) -> f32 {
        unsafe { self.packed }
    }

    #[must_use]
    pub fn r(&self) -> u8 {
        unsafe { self.unpacked[2] }
    }

    #[must_use]
    pub fn g(&self) -> u8 {
        unsafe { self.unpacked[1] }
    }

    #[must_use]
    pub fn b(&self) -> u8 {
        unsafe { self.unpacked[0] }
    }

    pub fn set_r(&mut self, r: u8) {
        unsafe { self.unpacked[2] = r }
    }

    pub fn set_g(&mut self, g: u8) {
        unsafe { self.unpacked[1] = g }
    }

    pub fn set_b(&mut self, b: u8) {
        unsafe { self.unpacked[0] = b }
    }
}

impl From<RGB> for f32 {
    fn from(rgb: RGB) -> Self {
        unsafe { rgb.packed }
    }
}

impl From<f32> for RGB {
    fn from(packed: f32) -> Self {
        RGB::new_from_packed_f32(packed)
    }
}

/// 3D point with x, y, z coordinates, commonly used in ROS with PCL.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[repr(C, align(16))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PointXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[cfg(feature = "nalgebra")]
#[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
impl From<nalgebra::Point3<f32>> for PointXYZ {
    fn from(point: nalgebra::Point3<f32>) -> Self {
        Self {
            x: point.x,
            y: point.y,
            z: point.z,
        }
    }
}

#[cfg(feature = "nalgebra")]
#[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
impl From<&nalgebra::Point3<f32>> for PointXYZ {
    fn from(point: &nalgebra::Point3<f32>) -> Self {
        Self {
            x: point.x,
            y: point.y,
            z: point.z,
        }
    }
}

#[cfg(feature = "nalgebra")]
#[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
impl From<nalgebra::Point3<f64>> for PointXYZ {
    fn from(point: nalgebra::Point3<f64>) -> Self {
        Self {
            x: point.x as f32,
            y: point.y as f32,
            z: point.z as f32,
        }
    }
}

#[cfg(feature = "nalgebra")]
#[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
impl From<&nalgebra::Point3<f64>> for PointXYZ {
    fn from(point: &nalgebra::Point3<f64>) -> Self {
        Self {
            x: point.x as f32,
            y: point.y as f32,
            z: point.z as f32,
        }
    }
}

#[cfg(feature = "nalgebra")]
#[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
impl From<PointXYZ> for nalgebra::Point3<f32> {
    fn from(point: PointXYZ) -> Self {
        nalgebra::Point3::new(point.x, point.y, point.z)
    }
}

#[cfg(feature = "nalgebra")]
#[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
impl From<&PointXYZ> for nalgebra::Point3<f32> {
    fn from(point: &PointXYZ) -> Self {
        nalgebra::Point3::new(point.x, point.y, point.z)
    }
}

#[cfg(feature = "nalgebra")]
#[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
impl From<PointXYZ> for nalgebra::Point3<f64> {
    fn from(point: PointXYZ) -> Self {
        nalgebra::Point3::new(point.x as f64, point.y as f64, point.z as f64)
    }
}

#[cfg(feature = "nalgebra")]
#[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
impl From<&PointXYZ> for nalgebra::Point3<f64> {
    fn from(point: &PointXYZ) -> Self {
        nalgebra::Point3::new(point.x as f64, point.y as f64, point.z as f64)
    }
}

impl PointXYZ {
    #[must_use]
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Get the coordinates as a nalgebra Point3.
    #[cfg(feature = "nalgebra")]
    #[deprecated(since = "0.5.2", note = "please use `xyz_f32` instead")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz(&self) -> nalgebra::Point3<f32> {
        self.xyz_f32()
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f32(&self) -> nalgebra::Point3<f32> {
        self.into()
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f64(&self) -> nalgebra::Point3<f64> {
        self.into()
    }
}

unsafe impl Send for PointXYZ {}
unsafe impl Sync for PointXYZ {}

impl From<RPCL2Point<3>> for PointXYZ {
    fn from(point: RPCL2Point<3>) -> Self {
        Self::new(point[0].get(), point[1].get(), point[2].get())
    }
}

impl From<PointXYZ> for RPCL2Point<3> {
    fn from(point: PointXYZ) -> Self {
        [point.x.into(), point.y.into(), point.z.into()].into()
    }
}

unsafe impl PointConvertible<3> for PointXYZ {
    fn layout() -> LayoutDescription {
        LayoutDescription::new(&[
            LayoutField::new("x", "f32", 4),
            LayoutField::new("y", "f32", 4),
            LayoutField::new("z", "f32", 4),
            LayoutField::padding(4),
        ])
    }
}

/// 3D point with x, y, z coordinates and an intensity value, commonly used in ROS with PCL.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[repr(C, align(16))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

impl PointXYZI {
    pub fn new(x: f32, y: f32, z: f32, intensity: f32) -> Self {
        Self { x, y, z, intensity }
    }

    /// Get the coordinates as a nalgebra Point3.
    #[cfg(feature = "nalgebra")]
    #[deprecated(since = "0.5.2", note = "please use `xyz_f32` instead")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz(&self) -> nalgebra::Point3<f32> {
        self.xyz_f32()
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f32(&self) -> nalgebra::Point3<f32> {
        nalgebra::Point3::new(self.x, self.y, self.z)
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f64(&self) -> nalgebra::Point3<f64> {
        nalgebra::Point3::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

unsafe impl Send for PointXYZI {}
unsafe impl Sync for PointXYZI {}

impl From<RPCL2Point<4>> for PointXYZI {
    fn from(point: RPCL2Point<4>) -> Self {
        Self::new(
            point[0].get(),
            point[1].get(),
            point[2].get(),
            point[3].get(),
        )
    }
}

impl From<PointXYZI> for RPCL2Point<4> {
    fn from(point: PointXYZI) -> Self {
        [
            point.x.into(),
            point.y.into(),
            point.z.into(),
            point.intensity.into(),
        ]
        .into()
    }
}

unsafe impl PointConvertible<4> for PointXYZI {
    fn layout() -> LayoutDescription {
        LayoutDescription::new(&[
            LayoutField::new("x", "f32", 4),
            LayoutField::new("y", "f32", 4),
            LayoutField::new("z", "f32", 4),
            LayoutField::new("intensity", "f32", 4),
        ])
    }
}

/// 3D point with x, y, z coordinates and a label, commonly used in ROS with PCL.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[repr(C, align(16))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PointXYZL {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub label: u32,
}

impl PointXYZL {
    pub fn new(x: f32, y: f32, z: f32, label: u32) -> Self {
        Self { x, y, z, label }
    }

    /// Get the coordinates as a nalgebra Point3.
    #[cfg(feature = "nalgebra")]
    #[deprecated(since = "0.5.2", note = "please use `xyz_f32` instead")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz(&self) -> nalgebra::Point3<f32> {
        self.xyz_f32()
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f32(&self) -> nalgebra::Point3<f32> {
        nalgebra::Point3::new(self.x, self.y, self.z)
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f64(&self) -> nalgebra::Point3<f64> {
        nalgebra::Point3::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

unsafe impl Send for PointXYZL {}
unsafe impl Sync for PointXYZL {}

impl From<RPCL2Point<4>> for PointXYZL {
    fn from(point: RPCL2Point<4>) -> Self {
        Self::new(
            point[0].get(),
            point[1].get(),
            point[2].get(),
            point[3].get(),
        )
    }
}

impl From<PointXYZL> for RPCL2Point<4> {
    fn from(point: PointXYZL) -> Self {
        [
            point.x.into(),
            point.y.into(),
            point.z.into(),
            point.label.into(),
        ]
        .into()
    }
}

unsafe impl PointConvertible<4> for PointXYZL {
    fn layout() -> LayoutDescription {
        LayoutDescription::new(&[
            LayoutField::new("x", "f32", 4),
            LayoutField::new("y", "f32", 4),
            LayoutField::new("z", "f32", 4),
            LayoutField::new("label", "u32", 4),
        ])
    }
}

/// 3D point with x, y, z coordinates and an RGB color value, commonly used in ROS with PCL.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[repr(C, align(16))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PointXYZRGB {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub rgb: RGB,
}

impl PointXYZRGB {
    #[must_use]
    pub fn new(x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> Self {
        let rgb = RGB::new(r, g, b);
        Self { x, y, z, rgb }
    }

    #[must_use]
    pub fn r(&self) -> u8 {
        self.rgb.r()
    }

    #[must_use]
    pub fn g(&self) -> u8 {
        self.rgb.g()
    }

    #[must_use]
    pub fn b(&self) -> u8 {
        self.rgb.b()
    }

    /// Get the coordinates as a nalgebra Point3.
    #[cfg(feature = "nalgebra")]
    #[deprecated(since = "0.5.2", note = "please use `xyz_f32` instead")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz(&self) -> nalgebra::Point3<f32> {
        self.xyz_f32()
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f32(&self) -> nalgebra::Point3<f32> {
        nalgebra::Point3::new(self.x, self.y, self.z)
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f64(&self) -> nalgebra::Point3<f64> {
        nalgebra::Point3::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

unsafe impl Send for PointXYZRGB {}
unsafe impl Sync for PointXYZRGB {}

impl From<RPCL2Point<4>> for PointXYZRGB {
    fn from(point: RPCL2Point<4>) -> Self {
        Self {
            x: point[0].get(),
            y: point[1].get(),
            z: point[2].get(),
            rgb: point[3].get(),
        }
    }
}

impl From<PointXYZRGB> for RPCL2Point<4> {
    fn from(point: PointXYZRGB) -> Self {
        [
            point.x.into(),
            point.y.into(),
            point.z.into(),
            f32::from(point.rgb).into(),
        ]
        .into()
    }
}

unsafe impl PointConvertible<4> for PointXYZRGB {
    fn layout() -> LayoutDescription {
        LayoutDescription::new(&[
            LayoutField::new("x", "f32", 4),
            LayoutField::new("y", "f32", 4),
            LayoutField::new("z", "f32", 4),
            LayoutField::new("rgb", "RGB", 4),
        ])
    }
}

/// 3D point with x, y, z coordinates and an RGBA color value, commonly used in ROS with PCL.
/// The alpha channel is commonly used as padding but this crate uses every channel and no padding.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[repr(C, align(16))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PointXYZRGBA {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub rgb: RGB,
    pub a: u8,
}

impl PointXYZRGBA {
    #[must_use]
    pub fn new(x: f32, y: f32, z: f32, r: u8, g: u8, b: u8, a: u8) -> Self {
        let rgb = RGB::new(r, g, b);
        Self { x, y, z, rgb, a }
    }

    #[must_use]
    pub fn r(&self) -> u8 {
        self.rgb.r()
    }

    #[must_use]
    pub fn g(&self) -> u8 {
        self.rgb.g()
    }

    #[must_use]
    pub fn b(&self) -> u8 {
        self.rgb.b()
    }

    /// Get the coordinates as a nalgebra Point3.
    #[cfg(feature = "nalgebra")]
    #[deprecated(since = "0.5.2", note = "please use `xyz_f32` instead")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz(&self) -> nalgebra::Point3<f32> {
        self.xyz_f32()
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f32(&self) -> nalgebra::Point3<f32> {
        nalgebra::Point3::new(self.x, self.y, self.z)
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f64(&self) -> nalgebra::Point3<f64> {
        nalgebra::Point3::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

unsafe impl Send for PointXYZRGBA {}
unsafe impl Sync for PointXYZRGBA {}

impl From<RPCL2Point<5>> for PointXYZRGBA {
    fn from(point: RPCL2Point<5>) -> Self {
        Self {
            x: point[0].get(),
            y: point[1].get(),
            z: point[2].get(),
            rgb: point[3].get::<f32>().into(),
            a: point[4].get(),
        }
    }
}

impl From<PointXYZRGBA> for RPCL2Point<5> {
    fn from(point: PointXYZRGBA) -> Self {
        [
            point.x.into(),
            point.y.into(),
            point.z.into(),
            f32::from(point.rgb).into(),
            point.a.into(),
        ]
        .into()
    }
}

unsafe impl PointConvertible<5> for PointXYZRGBA {
    fn layout() -> LayoutDescription {
        LayoutDescription::new(&[
            LayoutField::new("x", "f32", 4),
            LayoutField::new("y", "f32", 4),
            LayoutField::new("z", "f32", 4),
            LayoutField::new("rgb", "RGB", 4),
            LayoutField::new("a", "u8", 1),
            LayoutField::padding(15),
        ])
    }
}

/// 3D point with x, y, z coordinates, an RGB color value and a normal vector, commonly used in ROS with PCL.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[repr(C, align(16))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PointXYZRGBNormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub rgb: RGB,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
}

impl PointXYZRGBNormal {
    #[must_use]
    pub fn new(
        x: f32,
        y: f32,
        z: f32,
        rgb: RGB,
        normal_x: f32,
        normal_y: f32,
        normal_z: f32,
    ) -> Self {
        Self {
            x,
            y,
            z,
            rgb,
            normal_x,
            normal_y,
            normal_z,
        }
    }

    #[must_use]
    pub fn r(&self) -> u8 {
        self.rgb.r()
    }

    #[must_use]
    pub fn g(&self) -> u8 {
        self.rgb.g()
    }

    #[must_use]
    pub fn b(&self) -> u8 {
        self.rgb.b()
    }

    /// Get the coordinates as a nalgebra Point3.
    #[cfg(feature = "nalgebra")]
    #[deprecated(since = "0.5.2", note = "please use `xyz_f32` instead")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz(&self) -> nalgebra::Point3<f32> {
        self.xyz_f32()
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f32(&self) -> nalgebra::Point3<f32> {
        nalgebra::Point3::new(self.x, self.y, self.z)
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f64(&self) -> nalgebra::Point3<f64> {
        nalgebra::Point3::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

unsafe impl Send for PointXYZRGBNormal {}
unsafe impl Sync for PointXYZRGBNormal {}

impl From<RPCL2Point<7>> for PointXYZRGBNormal {
    fn from(point: RPCL2Point<7>) -> Self {
        Self {
            x: point[0].get(),
            y: point[1].get(),
            z: point[2].get(),
            rgb: point[3].get::<f32>().into(),
            normal_x: point[4].get(),
            normal_y: point[5].get(),
            normal_z: point[6].get(),
        }
    }
}

impl From<PointXYZRGBNormal> for RPCL2Point<7> {
    fn from(point: PointXYZRGBNormal) -> Self {
        [
            point.x.into(),
            point.y.into(),
            point.z.into(),
            f32::from(point.rgb).into(),
            point.normal_x.into(),
            point.normal_y.into(),
            point.normal_z.into(),
        ]
        .into()
    }
}

unsafe impl PointConvertible<7> for PointXYZRGBNormal {
    fn layout() -> LayoutDescription {
        LayoutDescription::new(&[
            LayoutField::new("x", "f32", 4),
            LayoutField::new("y", "f32", 4),
            LayoutField::new("z", "f32", 4),
            LayoutField::new("rgb", "RGB", 4),
            LayoutField::new("normal_x", "f32", 4),
            LayoutField::new("normal_y", "f32", 4),
            LayoutField::new("normal_z", "f32", 4),
            LayoutField::padding(4),
        ])
    }
}

/// 3D point with x, y, z coordinates, an intensity value and a normal vector, commonly used in ROS with PCL.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[repr(C, align(16))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PointXYZINormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
}

impl PointXYZINormal {
    #[must_use]
    pub fn new(
        x: f32,
        y: f32,
        z: f32,
        intensity: f32,
        normal_x: f32,
        normal_y: f32,
        normal_z: f32,
    ) -> Self {
        Self {
            x,
            y,
            z,
            intensity,
            normal_x,
            normal_y,
            normal_z,
        }
    }

    /// Get the coordinates as a nalgebra Point3.
    #[cfg(feature = "nalgebra")]
    #[deprecated(since = "0.5.2", note = "please use `xyz_f32` instead")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz(&self) -> nalgebra::Point3<f32> {
        self.xyz_f32()
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f32(&self) -> nalgebra::Point3<f32> {
        nalgebra::Point3::new(self.x, self.y, self.z)
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f64(&self) -> nalgebra::Point3<f64> {
        nalgebra::Point3::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

unsafe impl Send for PointXYZINormal {}
unsafe impl Sync for PointXYZINormal {}

impl From<RPCL2Point<7>> for PointXYZINormal {
    fn from(point: RPCL2Point<7>) -> Self {
        Self::new(
            point[0].get(),
            point[1].get(),
            point[2].get(),
            point[3].get(),
            point[4].get(),
            point[5].get(),
            point[6].get(),
        )
    }
}

impl From<PointXYZINormal> for RPCL2Point<7> {
    fn from(point: PointXYZINormal) -> Self {
        [
            point.x.into(),
            point.y.into(),
            point.z.into(),
            point.intensity.into(),
            point.normal_x.into(),
            point.normal_y.into(),
            point.normal_z.into(),
        ]
        .into()
    }
}

unsafe impl PointConvertible<7> for PointXYZINormal {
    fn layout() -> LayoutDescription {
        LayoutDescription::new(&[
            LayoutField::new("x", "f32", 4),
            LayoutField::new("y", "f32", 4),
            LayoutField::new("z", "f32", 4),
            LayoutField::new("intensity", "f32", 4),
            LayoutField::new("normal_x", "f32", 4),
            LayoutField::new("normal_y", "f32", 4),
            LayoutField::new("normal_z", "f32", 4),
            LayoutField::padding(4),
        ])
    }
}

/// 3D point with x, y, z coordinates and a label, commonly used in ROS with PCL.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[repr(C, align(16))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PointXYZRGBL {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub rgb: RGB,
    pub label: u32,
}

unsafe impl Send for PointXYZRGBL {}
unsafe impl Sync for PointXYZRGBL {}

impl PointXYZRGBL {
    #[must_use]
    pub fn new(x: f32, y: f32, z: f32, r: u8, g: u8, b: u8, label: u32) -> Self {
        let rgb = RGB::new(r, g, b);
        Self {
            x,
            y,
            z,
            rgb,
            label,
        }
    }

    #[must_use]
    pub fn r(&self) -> u8 {
        self.rgb.r()
    }

    #[must_use]
    pub fn g(&self) -> u8 {
        self.rgb.g()
    }

    #[must_use]
    pub fn b(&self) -> u8 {
        self.rgb.b()
    }

    /// Get the coordinates as a nalgebra Point3.
    #[cfg(feature = "nalgebra")]
    #[deprecated(since = "0.5.2", note = "please use `xyz_f32` instead")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz(&self) -> nalgebra::Point3<f32> {
        self.xyz_f32()
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f32(&self) -> nalgebra::Point3<f32> {
        nalgebra::Point3::new(self.x, self.y, self.z)
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f64(&self) -> nalgebra::Point3<f64> {
        nalgebra::Point3::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

impl From<RPCL2Point<5>> for PointXYZRGBL {
    fn from(point: RPCL2Point<5>) -> Self {
        Self {
            x: point[0].get(),
            y: point[1].get(),
            z: point[2].get(),
            rgb: point[3].get::<f32>().into(),
            label: point[4].get(),
        }
    }
}

impl From<PointXYZRGBL> for RPCL2Point<5> {
    fn from(point: PointXYZRGBL) -> Self {
        [
            point.x.into(),
            point.y.into(),
            point.z.into(),
            f32::from(point.rgb).into(),
            point.label.into(),
        ]
        .into()
    }
}

unsafe impl PointConvertible<5> for PointXYZRGBL {
    fn layout() -> LayoutDescription {
        LayoutDescription::new(&[
            LayoutField::new("x", "f32", 4),
            LayoutField::new("y", "f32", 4),
            LayoutField::new("z", "f32", 4),
            LayoutField::new("rgb", "RGB", 4),
            LayoutField::new("label", "u32", 4),
            LayoutField::padding(12),
        ])
    }
}

/// 3D point with x, y, z coordinates and a normal vector, commonly used in ROS with PCL.
#[derive(Clone, Debug, PartialEq, Copy, Default)]
#[repr(C, align(16))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PointXYZNormal {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
}

impl PointXYZNormal {
    #[must_use]
    pub fn new(x: f32, y: f32, z: f32, normal_x: f32, normal_y: f32, normal_z: f32) -> Self {
        Self {
            x,
            y,
            z,
            normal_x,
            normal_y,
            normal_z,
        }
    }

    /// Get the coordinates as a nalgebra Point3.
    #[cfg(feature = "nalgebra")]
    #[deprecated(since = "0.5.2", note = "please use `xyz_f32` instead")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz(&self) -> nalgebra::Point3<f32> {
        self.xyz_f32()
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f32(&self) -> nalgebra::Point3<f32> {
        nalgebra::Point3::new(self.x, self.y, self.z)
    }

    #[cfg(feature = "nalgebra")]
    #[cfg_attr(docsrs, doc(cfg(feature = "nalgebra")))]
    pub fn xyz_f64(&self) -> nalgebra::Point3<f64> {
        nalgebra::Point3::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

unsafe impl Send for PointXYZNormal {}
unsafe impl Sync for PointXYZNormal {}

impl From<RPCL2Point<6>> for PointXYZNormal {
    fn from(point: RPCL2Point<6>) -> Self {
        Self::new(
            point[0].get(),
            point[1].get(),
            point[2].get(),
            point[3].get(),
            point[4].get(),
            point[5].get(),
        )
    }
}

impl From<PointXYZNormal> for RPCL2Point<6> {
    fn from(point: PointXYZNormal) -> Self {
        [
            point.x.into(),
            point.y.into(),
            point.z.into(),
            point.normal_x.into(),
            point.normal_y.into(),
            point.normal_z.into(),
        ]
        .into()
    }
}

unsafe impl PointConvertible<6> for PointXYZNormal {
    fn layout() -> LayoutDescription {
        LayoutDescription::new(&[
            LayoutField::new("x", "f32", 4),
            LayoutField::new("y", "f32", 4),
            LayoutField::new("z", "f32", 4),
            LayoutField::new("normal_x", "f32", 4),
            LayoutField::new("normal_y", "f32", 4),
            LayoutField::new("normal_z", "f32", 4),
            LayoutField::padding(8),
        ])
    }
}
