use std::str::FromStr;

use crate::*;

/// Datatypes from the [PointField message](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html).
#[derive(Default, Clone, Debug, PartialEq, Copy)]
pub enum FieldDatatype {
    F32,
    F64,
    I32,
    U8,
    U16,
    #[default]
    U32,
    I8,
    I16,

    /// While RGB is not officially supported by ROS, it is used in the tooling as a packed f32.
    /// To make it easy to work with and avoid packing code, the [`ros_pointcloud2::points::RGB`] union is supported here and handled like a f32.
    RGB,
}

impl FieldDatatype {
    pub fn size(&self) -> usize {
        match self {
            FieldDatatype::U8 => std::mem::size_of::<u8>(),
            FieldDatatype::U16 => std::mem::size_of::<u16>(),
            FieldDatatype::U32 => std::mem::size_of::<u32>(),
            FieldDatatype::I8 => std::mem::size_of::<i8>(),
            FieldDatatype::I16 => std::mem::size_of::<i16>(),
            FieldDatatype::I32 => std::mem::size_of::<i32>(),
            FieldDatatype::F32 => std::mem::size_of::<f32>(),
            FieldDatatype::F64 => std::mem::size_of::<f64>(),
            FieldDatatype::RGB => std::mem::size_of::<f32>(), // packed in f32
        }
    }
}

impl FromStr for FieldDatatype {
    type Err = MsgConversionError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "f32" => Ok(FieldDatatype::F32),
            "f64" => Ok(FieldDatatype::F64),
            "i32" => Ok(FieldDatatype::I32),
            "u8" => Ok(FieldDatatype::U8),
            "u16" => Ok(FieldDatatype::U16),
            "u32" => Ok(FieldDatatype::U32),
            "i8" => Ok(FieldDatatype::I8),
            "i16" => Ok(FieldDatatype::I16),
            "rgb" => Ok(FieldDatatype::RGB),
            _ => Err(MsgConversionError::UnsupportedFieldType(s.into())),
        }
    }
}

/// Getter trait for the datatype of a field value.
pub trait GetFieldDatatype {
    fn field_datatype() -> FieldDatatype;
}

impl GetFieldDatatype for f32 {
    fn field_datatype() -> FieldDatatype {
        FieldDatatype::F32
    }
}

impl GetFieldDatatype for f64 {
    fn field_datatype() -> FieldDatatype {
        FieldDatatype::F64
    }
}

impl GetFieldDatatype for i32 {
    fn field_datatype() -> FieldDatatype {
        FieldDatatype::I32
    }
}

impl GetFieldDatatype for u8 {
    fn field_datatype() -> FieldDatatype {
        FieldDatatype::U8
    }
}

impl GetFieldDatatype for u16 {
    fn field_datatype() -> FieldDatatype {
        FieldDatatype::U16
    }
}

impl GetFieldDatatype for u32 {
    fn field_datatype() -> FieldDatatype {
        FieldDatatype::U32
    }
}

impl GetFieldDatatype for i8 {
    fn field_datatype() -> FieldDatatype {
        FieldDatatype::I8
    }
}

impl GetFieldDatatype for i16 {
    fn field_datatype() -> FieldDatatype {
        FieldDatatype::I16
    }
}

/// Convenience implementation for the RGB union.
impl GetFieldDatatype for crate::points::RGB {
    fn field_datatype() -> FieldDatatype {
        FieldDatatype::RGB
    }
}

impl TryFrom<u8> for FieldDatatype {
    type Error = MsgConversionError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(FieldDatatype::I8),
            2 => Ok(FieldDatatype::U8),
            3 => Ok(FieldDatatype::I16),
            4 => Ok(FieldDatatype::U16),
            5 => Ok(FieldDatatype::I32),
            6 => Ok(FieldDatatype::U32),
            7 => Ok(FieldDatatype::F32),
            8 => Ok(FieldDatatype::F64),
            _ => Err(MsgConversionError::UnsupportedFieldType(value.to_string())),
        }
    }
}

impl From<FieldDatatype> for u8 {
    fn from(val: FieldDatatype) -> Self {
        match val {
            FieldDatatype::I8 => 1,
            FieldDatatype::U8 => 2,
            FieldDatatype::I16 => 3,
            FieldDatatype::U16 => 4,
            FieldDatatype::I32 => 5,
            FieldDatatype::U32 => 6,
            FieldDatatype::F32 => 7,
            FieldDatatype::F64 => 8,
            FieldDatatype::RGB => 7, // RGB is marked as f32 in the buffer
        }
    }
}

impl TryFrom<&ros_types::PointFieldMsg> for FieldDatatype {
    type Error = MsgConversionError;

    fn try_from(value: &ros_types::PointFieldMsg) -> Result<Self, Self::Error> {
        Self::try_from(value.datatype)
    }
}

/// Matching field names from each data point.
/// Always make sure to use the same order as in your conversion implementation to have a correct mapping.
///
/// This trait is needed to implement the `PointConvertible` trait.
///
/// # Example
/// ```
/// use ros_pointcloud2::prelude::*;
///
/// #[derive(Clone, Debug, PartialEq, Copy)]
/// pub struct MyPointXYZI {
///     pub x: f32,
///     pub y: f32,
///     pub z: f32,
///     pub intensity: f32,
/// }
///
/// impl Fields<4> for MyPointXYZI {
///    fn field_names_ordered() -> [&'static str; 4] {
///       ["x", "y", "z", "intensity"]
///   }
/// }
/// ```
pub trait Fields<const N: usize> {
    fn field_names_ordered() -> [&'static str; N];
}

pub struct PointDataBuffer([u8; 8]);

impl std::ops::Index<usize> for PointDataBuffer {
    type Output = u8;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl PointDataBuffer {
    pub fn new(data: [u8; 8]) -> Self {
        Self(data)
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }

    pub fn raw(self) -> [u8; 8] {
        self.0
    }

    pub fn from_slice(data: &[u8]) -> Self {
        let mut buffer = [0; 8];
        data.iter().enumerate().for_each(|(i, &v)| buffer[i] = v);
        Self(buffer)
    }
}

impl From<&[u8]> for PointDataBuffer {
    fn from(data: &[u8]) -> Self {
        Self::from_slice(data)
    }
}

impl<const N: usize> From<[u8; N]> for PointDataBuffer {
    fn from(data: [u8; N]) -> Self {
        Self::from(data.as_slice())
    }
}

impl From<i8> for PointDataBuffer {
    fn from(x: i8) -> Self {
        x.to_le_bytes().into()
    }
}

impl From<i16> for PointDataBuffer {
    fn from(x: i16) -> Self {
        x.to_le_bytes().into()
    }
}

impl From<u16> for PointDataBuffer {
    fn from(x: u16) -> Self {
        x.to_le_bytes().into()
    }
}

impl From<i32> for PointDataBuffer {
    fn from(x: i32) -> Self {
        x.to_le_bytes().into()
    }
}

impl From<u32> for PointDataBuffer {
    fn from(x: u32) -> Self {
        x.to_le_bytes().into()
    }
}

impl From<f32> for PointDataBuffer {
    fn from(x: f32) -> Self {
        x.to_le_bytes().into()
    }
}

impl From<f64> for PointDataBuffer {
    fn from(x: f64) -> Self {
        x.to_le_bytes().into()
    }
}

impl From<u8> for PointDataBuffer {
    fn from(x: u8) -> Self {
        x.to_le_bytes().into()
    }
}

impl From<points::RGB> for PointDataBuffer {
    fn from(x: points::RGB) -> Self {
        x.raw().to_le_bytes().into()
    }
}

/// This trait is used to convert a byte slice to a primitive type.
/// All PointField types are supported.
pub trait FromBytes: Default + Sized + Copy + GetFieldDatatype + Into<PointDataBuffer> {
    fn from_be_bytes(bytes: PointDataBuffer) -> Self;
    fn from_le_bytes(bytes: PointDataBuffer) -> Self;
}

impl FromBytes for i8 {
    fn from_be_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_be_bytes([bytes[0]])
    }

    fn from_le_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_le_bytes([bytes[0]])
    }
}

impl FromBytes for i16 {
    fn from_be_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1]])
    }

    fn from_le_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1]])
    }
}

impl FromBytes for u16 {
    fn from_be_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1]])
    }

    fn from_le_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1]])
    }
}

impl FromBytes for u32 {
    fn from_be_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    fn from_le_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }
}

impl FromBytes for f32 {
    fn from_be_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    fn from_le_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }
}

impl FromBytes for points::RGB {
    fn from_be_bytes(bytes: PointDataBuffer) -> Self {
        Self::new_from_packed_f32(f32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    fn from_le_bytes(bytes: PointDataBuffer) -> Self {
        Self::new_from_packed_f32(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }
}

impl FromBytes for i32 {
    #[inline]
    fn from_be_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }
    fn from_le_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }
}

impl FromBytes for f64 {
    fn from_be_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_be_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ])
    }

    fn from_le_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_le_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ])
    }
}

impl FromBytes for u8 {
    fn from_be_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_be_bytes([bytes[0]])
    }

    fn from_le_bytes(bytes: PointDataBuffer) -> Self {
        Self::from_le_bytes([bytes[0]])
    }
}

pub enum ByteSimilarity {
    Equal,
    Overlapping,
    Different,
}

#[derive(Default, Clone, Debug, PartialEq, Copy)]
pub enum Endian {
    Big,
    #[default]
    Little,
}

#[derive(Default, Clone, Debug, PartialEq, Copy)]
pub enum Denseness {
    #[default]
    Dense,
    Sparse,
}
