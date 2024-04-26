use crate::*;

/// Datatypes from the [PointField message](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html).
#[derive(Clone, Debug, PartialEq, Copy)]
pub enum FieldDatatype {
    F32,
    F64,
    I32,
    U8,
    U16,
    U32,
    I8,
    I16,
}

impl Default for FieldDatatype {
    fn default() -> Self {
        FieldDatatype::F32
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

pub(crate) fn convert_to_msg_code(geo_type: &FieldDatatype) -> u8 {
    match geo_type {
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

pub(crate) fn convert_msg_code_to_type(code: u8) -> Result<FieldDatatype, ConversionError> {
    match code {
        7 => Ok(FieldDatatype::F32),
        8 => Ok(FieldDatatype::F64),
        5 => Ok(FieldDatatype::I32),
        2 => Ok(FieldDatatype::U8),
        4 => Ok(FieldDatatype::U16),
        6 => Ok(FieldDatatype::U32),
        1 => Ok(FieldDatatype::I8),
        3 => Ok(FieldDatatype::I16),
        _ => Err(ConversionError::UnsupportedFieldType),
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
            if field.datatype != convert_to_msg_code(xyz_field_type) {
                return Err(ConversionError::InvalidFieldFormat);
            }
            Ok(field.clone())
        }
        None => Err(ConversionError::NotEnoughFields),
    }
}

/// Matching field names from each meta data per point to the PointField name.
/// Always make sure to use the same order as in your Into<> implementation to have a correct mapping.
pub trait MetaNames<const METADIM: usize> {
    fn meta_names() -> [String; METADIM];
}

pub(crate) fn datatype_size(datatype: &FieldDatatype) -> usize {
    match datatype {
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

#[inline(always)]
pub(crate) fn add_point_to_byte_buffer<
    T: FromBytes,
    const SIZE: usize,
    const DIM: usize,
    const METADIM: usize,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
>(
    coords: C,
    cloud: &mut PointCloud2Msg,
) -> Result<bool, ConversionError> {
    let (coords_data, coords_meta): ([T; DIM], [PointMeta; METADIM]) = match coords.try_into() {
        Ok(meta) => meta,
        Err(_) => return Err(ConversionError::PointConversionError),
    };

    // (x, y, z...)
    coords_data
        .iter()
        .for_each(|x| cloud.data.extend_from_slice(T::bytes(x).as_slice()));

    // meta data description
    coords_meta.iter().for_each(|meta| {
        let truncated_bytes = &meta.bytes[0..datatype_size(&meta.datatype)];
        cloud.data.extend_from_slice(truncated_bytes);
    });

    cloud.width += 1;

    Ok(true)
}

/// This trait is used to convert a byte slice to a primitive type.
/// All PointField types are supported.
pub trait FromBytes: Default + Sized + Copy + GetFieldDatatype {
    fn from_be_bytes(bytes: &[u8]) -> Self;
    fn from_le_bytes(bytes: &[u8]) -> Self;

    fn bytes(_: &Self) -> Vec<u8>;
}

impl FromBytes for i8 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0]])
    }

    fn bytes(x: &i8) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for i16 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1]])
    }

    fn bytes(x: &i16) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for u16 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1]])
    }

    fn bytes(x: &u16) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for u32 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    fn bytes(x: &u32) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for f32 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    fn bytes(x: &f32) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for i32 {
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
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ])
    }

    fn bytes(x: &f64) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for u8 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0]])
    }

    fn bytes(x: &u8) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Endianness {
    Big,
    Little,
}

impl Default for Endianness {
    fn default() -> Self {
        Endianness::Little
    }
}

pub(crate) fn load_loadable<T: FromBytes, const SIZE: usize>(
    start_idx: usize,
    data: &[u8],
    endian: &Endianness,
) -> Option<T> {
    match endian {
        Endianness::Big => Some(T::from_be_bytes(
            load_bytes::<SIZE>(start_idx, data)?.as_slice(),
        )),
        Endianness::Little => Some(T::from_le_bytes(
            load_bytes::<SIZE>(start_idx, data)?.as_slice(),
        )),
    }
}

fn load_bytes<const S: usize>(start_idx: usize, data: &[u8]) -> Option<[u8; S]> {
    if start_idx + S > data.len() {
        return None;
    }
    let mut buff: [u8; S] = [0; S];
    for (byte, buff_val) in buff.iter_mut().enumerate().take(S) {
        let raw_byte = data.get(start_idx + byte);
        match raw_byte {
            None => {
                return None;
            }
            Some(some_byte) => {
                *buff_val = *some_byte;
            }
        }
    }

    Some(buff)
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
