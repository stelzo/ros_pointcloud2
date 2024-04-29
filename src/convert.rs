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
}

impl FieldDatatype {
    pub fn size(&self) -> usize {
        match self {
            FieldDatatype::U8 => 1,
            FieldDatatype::U16 => 2,
            FieldDatatype::U32 => 4,
            FieldDatatype::I8 => 1,
            FieldDatatype::I16 => 2,
            FieldDatatype::I32 => 4,
            FieldDatatype::F32 => 4,
            FieldDatatype::F64 => 8,
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

impl TryFrom<u8> for FieldDatatype {
    type Error = ConversionError;

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
            _ => Err(ConversionError::UnsupportedFieldType),
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
        }
    }
}

pub(crate) fn check_coord(
    coord: Option<usize>,
    fields: &[PointFieldMsg],
    xyz_field_type: &FieldDatatype,
) -> Result<PointFieldMsg, ConversionError> {
    match coord {
        Some(y_idx) => {
            let field = &fields[y_idx];
            if field.datatype != u8::from(*xyz_field_type) {
                return Err(ConversionError::InvalidFieldFormat);
            }
            Ok(field.clone())
        }
        None => Err(ConversionError::NotEnoughFields),
    }
}

/// Matching field names from each meta data per point to the PointField name.
/// Always make sure to use the same order as in your conversion implementation to have a correct mapping.
///
/// This trait is needed to implement the `PointConvertible` trait.
///
/// # Example for full point conversion.
/// ```
/// use ros_pointcloud2::{Point, PointConvertible, MetaNames, size_of};
///
/// #[derive(Clone, Debug, PartialEq, Copy)]
/// pub struct MyPointXYZI {
///     pub x: f32,
///     pub y: f32,
///     pub z: f32,
///     pub intensity: f32,
/// }
///
/// impl MetaNames<1> for MyPointXYZI {
///    fn meta_names() -> [&'static str; 1] {
///       ["intensity"]
///   }
/// }
/// ```
pub trait MetaNames<const METADIM: usize> {
    fn meta_names() -> [&'static str; METADIM];
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

#[derive(Default, Clone, Debug, PartialEq)]
pub(crate) enum Endianness {
    Big,

    #[default]
    Little,
}

#[inline(always)]
pub(crate) fn load_loadable<T: FromBytes, const SIZE: usize>(
    start_idx: usize,
    data: &[u8],
    endian: &Endianness,
) -> T {
    match endian {
        Endianness::Big => T::from_be_bytes(load_bytes::<SIZE>(start_idx, data).as_slice()),
        Endianness::Little => T::from_le_bytes(load_bytes::<SIZE>(start_idx, data).as_slice()),
    }
}

/// Note: check if the data slice is long enough to load the bytes beforehand! Uses unsafe indexing.
#[inline(always)]
fn load_bytes<const S: usize>(start_idx: usize, data: &[u8]) -> [u8; S] {
    let mut target = [u8::default(); S];

    debug_assert!(target.len() == S);
    debug_assert!(data.len() >= start_idx + S);

    let source = unsafe { data.get_unchecked(start_idx..start_idx + S) };
    target
        .iter_mut()
        .zip(source.iter())
        .for_each(|(target, source)| {
            *target = *source;
        });
    target
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
