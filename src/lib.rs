//#![doc = include_str!("../README.md")]
pub mod convert;
pub mod pcl_utils;
pub mod ros_types;

/// Macro to get the size of a type at compile time. This is a convenience macro to avoid writing out the full std::mem::size_of::<T>().
/// Use it for your custom Reader and Writer implementations.
/// # Example
/// ```
/// use ros_pointcloud2::{
///     writer::Writer, size_of, pcl_utils::PointXYZ,
/// };
///
/// type MyWriterXYZ = Writer<f32, { size_of!(f32) }, 3, 0, PointXYZ>;
/// ```
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

/// All errors that can occur for creating Reader and Writer.
#[derive(Debug)]
pub enum ConversionError {
    /// The coordinate field does not match the expected datatype.
    InvalidFieldFormat,

    /// Not enough meta or dimensional fields in the PointCloud2 message.
    NotEnoughFields,

    /// The dimensionality of the point inside the message is too high.
    TooManyDimensions,

    /// The field type is not supported by the ROS message description.
    UnsupportedFieldType,

    /// There are no points in the point cloud.
    NoPoints,

    /// The length of the byte buffer in the message does not match the expected length computed from the fields.
    DataLengthMismatch,
}

pub struct Point<T, const DIM: usize, const METADIM: usize> {
    pub coords: [T; DIM],
    pub meta: [PointMeta; METADIM],
}

/// Trait to convert a point to a tuple of coordinates and meta data.
/// Implement this trait for your point type to use the conversion.
pub trait PointConvertible<T, const SIZE: usize, const DIM: usize, const METADIM: usize>:
    From<Point<T, DIM, METADIM>> + Into<Point<T, DIM, METADIM>> + MetaNames<METADIM> + Clone + 'static
where
    T: FromBytes,
{
}

/// Metadata representation for a point.
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
/// let my_data: f64 = meta.get();
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
        for (byte, save_byte) in raw_bytes.into_iter().zip(bytes.iter_mut()) {
            *save_byte = byte;
        }

        Self {
            bytes,
            datatype: T::field_datatype(),
        }
    }

    fn from_buffer(data: &[u8], offset: usize, datatype: &FieldDatatype) -> Self {
        debug_assert!(data.len() >= offset + datatype.size());

        let bytes = unsafe { data.get_unchecked(offset..offset + datatype.size()) };
        let mut bytes_array = [0; std::mem::size_of::<f64>()]; // 8 bytes as f64 is the largest type
        for (byte, save_byte) in bytes.into_iter().zip(bytes_array.iter_mut()) {
            *save_byte = *byte;
        }

        Self {
            bytes: bytes_array,
            datatype: *datatype,
        }
    }

    /// Get the numeric value from the PointMeta description.
    ///
    /// # Example
    /// ```
    /// let original_data: f64 = 1.0;
    /// let meta = ros_pointcloud2::PointMeta::new(original_data);
    /// let my_data: f64 = meta.get();
    /// ```
    pub fn get<T: FromBytes>(&self) -> T {
        let size = T::field_datatype().size();
        let bytes = self
            .bytes
            .get(0..size)
            .expect("Exceeds bounds of f64, which is the largest type");
        T::from_le_bytes(bytes)
    }
}

impl From<f32> for PointMeta {
    fn from(value: f32) -> Self {
        Self::new(value)
    }
}

impl From<f64> for PointMeta {
    fn from(value: f64) -> Self {
        Self::new(value)
    }
}

impl From<i32> for PointMeta {
    fn from(value: i32) -> Self {
        Self::new(value)
    }
}

impl From<u8> for PointMeta {
    fn from(value: u8) -> Self {
        Self::new(value)
    }
}

impl From<u16> for PointMeta {
    fn from(value: u16) -> Self {
        Self::new(value)
    }
}

impl From<u32> for PointMeta {
    fn from(value: u32) -> Self {
        Self::new(value)
    }
}

impl From<i8> for PointMeta {
    fn from(value: i8) -> Self {
        Self::new(value)
    }
}

impl From<i16> for PointMeta {
    fn from(value: i16) -> Self {
        Self::new(value)
    }
}
