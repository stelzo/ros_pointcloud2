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

    /// While RGB is not officially supported by ROS, it is used in practice as a packed f32.
    /// To make it easier to work with and avoid packing code, the
    /// [`ros_pointcloud2::points::RGB`] union is supported here and handled like a f32.
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

impl TryFrom<String> for FieldDatatype {
    type Error = MsgConversionError;

    fn try_from(value: String) -> Result<Self, Self::Error> {
        match value.to_lowercase().as_str() {
            "f32" => Ok(FieldDatatype::F32),
            "f64" => Ok(FieldDatatype::F64),
            "i32" => Ok(FieldDatatype::I32),
            "u8" => Ok(FieldDatatype::U8),
            "u16" => Ok(FieldDatatype::U16),
            "u32" => Ok(FieldDatatype::U32),
            "i8" => Ok(FieldDatatype::I8),
            "i16" => Ok(FieldDatatype::I16),
            "rgb" => Ok(FieldDatatype::RGB),
            _ => Err(MsgConversionError::UnsupportedFieldType(value)),
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

/// This trait is used to convert a byte slice to a primitive type.
/// All PointField types are supported.
pub trait FromBytes: Default + Sized + Copy + GetFieldDatatype {
    fn from_be_bytes(bytes: &[u8]) -> Self;
    fn from_le_bytes(bytes: &[u8]) -> Self;

    fn bytes(_: &Self) -> Vec<u8>;
}

impl FromBytes for i8 {
    #[inline]
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0]])
    }

    #[inline]
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0]])
    }

    #[inline]
    fn bytes(x: &i8) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for i16 {
    #[inline]
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1]])
    }

    #[inline]
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1]])
    }

    #[inline]
    fn bytes(x: &i16) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for u16 {
    #[inline]
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1]])
    }

    #[inline]
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1]])
    }

    #[inline]
    fn bytes(x: &u16) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for u32 {
    #[inline]
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    #[inline]
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    #[inline]
    fn bytes(x: &u32) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for f32 {
    #[inline]
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    #[inline]
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    #[inline]
    fn bytes(x: &f32) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for points::RGB {
    #[inline]
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::new_from_packed_f32(f32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    #[inline]
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::new_from_packed_f32(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    #[inline]
    fn bytes(x: &points::RGB) -> Vec<u8> {
        Vec::from(x.raw().to_le_bytes())
    }
}

impl FromBytes for i32 {
    #[inline]
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    fn bytes(x: &i32) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for f64 {
    #[inline]
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ])
    }

    #[inline]
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ])
    }

    #[inline]
    fn bytes(x: &f64) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for u8 {
    #[inline]
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0]])
    }

    #[inline]
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0]])
    }

    #[inline]
    fn bytes(x: &u8) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

#[derive(Default, Clone, Debug, PartialEq, Copy)]
pub enum Endianness {
    Big,

    #[default]
    Little,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_bytes() {
        i8::bytes(&1);
        u8::bytes(&1);
        i16::bytes(&1);
        u16::bytes(&1);
        i32::bytes(&1);
        u32::bytes(&1);
        f32::bytes(&1.0);
        f64::bytes(&1.0);
    }
}
