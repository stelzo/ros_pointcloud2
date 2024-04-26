// TODO test reading writing different types
// TODO read less info from the point cloud than is available
// TODO write less info to the point cloud than is available
// TODO explain templating arguments more for reader and writer

#![doc = include_str!("../README.md")]
pub mod convert;
pub mod pcl_utils;
pub mod ros_types;

#[macro_export]
macro_rules! size_of {
    ($t:ty) => {
        std::mem::size_of::<$t>()
    };
}

pub mod reader;
pub mod writer;

pub use ros_types::PointCloud2Msg;

use crate::convert::*;
use crate::pcl_utils::*;
use crate::ros_types::PointFieldMsg;

pub use fallible_iterator::FallibleIterator;

/// Errors that can occur when converting between PointCloud2 and a custom type.
#[derive(Debug)]
pub enum ConversionError {
    InvalidFieldFormat,
    NotEnoughFields,
    TooManyDimensions,
    UnsupportedFieldType,
    NoPoints,
    DataLengthMismatch,
    MetaIndexLengthMismatch,
    EndOfBuffer,
    PointConversionError,
    MetaDatatypeMismatch,
}

/// Trait to convert a point to a tuple of coordinates and meta data.
/// Implement this trait for your point type to use the conversion.
pub trait PointConvertible<T, const SIZE: usize, const DIM: usize, const METADIM: usize>:
    TryInto<([T; DIM], [PointMeta; METADIM])>
    + TryFrom<([T; DIM], [PointMeta; METADIM])>
    + MetaNames<METADIM>
    + Clone
    + 'static
where
    T: FromBytes,
{
}

/// Meta data representation for a point
///
/// This struct is used to store meta data in a fixed size byte buffer along the with the
/// datatype that is encoded so that it can be decoded later.
///
/// # Example
/// ```
/// use ros_pointcloud2::PointMeta;
///
/// let original_data: f64 = 1.0;
/// let meta = PointMeta::new(original_data);
/// let my_data: f64 = meta.get().unwrap();
/// ```
#[derive(Debug, Clone, Copy)]
pub struct PointMeta {
    bytes: [u8; std::mem::size_of::<f64>()],
    datatype: FieldDatatype,
}

impl Default for PointMeta {
    fn default() -> Self {
        Self {
            bytes: [0; std::mem::size_of::<f64>()],
            datatype: FieldDatatype::F32,
        }
    }
}

impl PointMeta {
    /// Create a new PointMeta from a value
    ///
    /// # Example
    /// ```
    /// let meta = ros_pointcloud2::PointMeta::new(1.0);
    /// ```
    pub fn new<T: FromBytes>(value: T) -> Self {
        let raw_bytes = T::bytes(&value);
        let mut bytes = [0; std::mem::size_of::<f64>()];
        for (idx, byte) in raw_bytes.iter().enumerate() {
            bytes[idx] = *byte;
        }
        Self {
            bytes,
            datatype: T::field_datatype(),
        }
    }

    fn new_from_buffer(
        data: &[u8],
        offset: usize,
        datatype: &FieldDatatype,
    ) -> Result<Self, ConversionError> {
        let size = datatype_size(datatype);
        let bytes = data
            .get(offset..offset + size)
            .ok_or(ConversionError::DataLengthMismatch)?;
        let mut bytes_array = [0; std::mem::size_of::<f64>()]; // 8 bytes as f64 is the largest type
        for (idx, byte) in bytes.iter().enumerate() {
            bytes_array[idx] = *byte;
        }
        Ok(Self {
            bytes: bytes_array,
            datatype: *datatype,
        })
    }

    /// Get the value from the PointMeta. It will return None if the datatype does not match.
    ///
    /// # Example
    /// ```
    /// let original_data: f64 = 1.0;
    /// let meta = ros_pointcloud2::PointMeta::new(original_data);
    /// let my_data: f64 = meta.get().unwrap();
    /// ```
    pub fn get<T: FromBytes>(&self) -> Result<T, ConversionError> {
        if self.datatype != T::field_datatype() {
            return Err(ConversionError::MetaDatatypeMismatch);
        }
        let size = datatype_size(&T::field_datatype());
        if let Some(bytes) = self.bytes.get(0..size) {
            Ok(T::from_le_bytes(bytes))
        } else {
            Err(ConversionError::DataLengthMismatch) // self.bytes are not long enough, already handled while parsing
        }
    }
}
