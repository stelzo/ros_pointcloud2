//! A library to work with the PointCloud2 message type from ROS.
//!
//! ros_pointcloud2 mainly provides a [`ros_pointcloud2::PointCloud2Msg`] type that implements
//! functions for conversion to and from iterators.
//!
//! - [`ros_pointcloud2::PointCloud2Msg::try_from_iter`]
//! - [`ros_pointcloud2::PointCloud2Msg::try_into_iter`]
//!
//! For ROS interoperability, the message type generated by the respective crate must be converted to
//! the [`ros_pointcloud2::PointCloud2Msg`] using the `From` trait,
//! which mostly does ownership transfers without copying the point data.
//!
//! There are implementations for the `r2r`, `rclrs` (ros2_rust) and `rosrust_msg` message types
//! available in the feature flags. If you miss a message type, please open an issue or a PR.
//! See the [`ros_pointcloud2::ros_types`] module for more information.
//!
//! Common point types like [`ros_pointcloud2::pcl_utils::PointXYZ`] or
//! [`ros_pointcloud2::pcl_utils::PointXYZI`] are provided in the
//! [`ros_pointcloud2::pcl_utils`] module. You can implement any custom point type
//! that can be described by the specification.
//!
//! # Example PointXYZ
//! ```
//! use ros_pointcloud2::prelude::*;
//!
//! // PointXYZ is predefined
//! let cloud_points = vec![
//!   PointXYZ::new(9.0006, 42.0, -6.2),
//!   PointXYZ::new(f32::MAX, f32::MIN, f32::MAX),
//! ];
//!
//! // For equality test later
//! let cloud_copy = cloud_points.clone();
//!
//! // Give the Vec or anything that implements `IntoIterator`.
//! let in_msg = PointCloud2Msg::try_from_iter(cloud_points).unwrap();
//!
//! // Convert to your ROS crate message type, we will use r2r here.
//! // let msg: r2r::sensor_msgs::msg::PointCloud2 = in_msg.into();
//! // Publish ...
//! // ... now incoming from a topic.
//! // let in_msg: PointCloud2Msg = msg.into();
//!
//! let new_pcl = in_msg.try_into_iter().unwrap()
//!     .map(|point: PointXYZ| { // Define the data you want from the point
//!         // Some logic here
//!
//!         point
//!     })
//!     .collect::<Vec<_>>(); // iterating points here O(n)
//!
//! assert_eq!(new_pcl, cloud_copy);
//! ```

pub mod convert;
pub mod points;
pub mod prelude;
pub mod ros_types;

pub mod iterator;

use crate::convert::{FieldDatatype, FromBytes};
use crate::ros_types::{HeaderMsg, PointFieldMsg};
use convert::Endianness;
pub use convert::Fields;

/// All errors that can occur while converting to or from the PointCloud2 message.
pub enum MsgConversionError {
    InvalidFieldFormat,
    NotEnoughFields,
    TooManyDimensions,
    UnsupportedFieldType(String),
    NoPoints,
    DataLengthMismatch,
    FieldNotFound(Vec<String>),
}

impl std::fmt::Display for MsgConversionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MsgConversionError::InvalidFieldFormat => {
                write!(f, "The field does not match the expected datatype.")
            }
            MsgConversionError::NotEnoughFields => {
                write!(f, "Not enough fields in the PointCloud2 message.")
            }
            MsgConversionError::TooManyDimensions => {
                write!(
                    f,
                    "The dimensionality of the point inside the message is too high."
                )
            }
            MsgConversionError::UnsupportedFieldType(datatype) => {
                write!(
                    f,
                    "The field datatype is not supported by the ROS message description: {}",
                    datatype
                )
            }
            MsgConversionError::NoPoints => {
                write!(f, "There are no points in the point cloud.")
            }
            MsgConversionError::DataLengthMismatch => {
                write!(f, "The length of the byte buffer in the message does not match the expected length computed from the fields.")
            }
            MsgConversionError::FieldNotFound(fields) => {
                write!(f, "There are fields missing in the message: {:?}", fields)
            }
        }
    }
}

impl std::fmt::Debug for MsgConversionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        <MsgConversionError as std::fmt::Display>::fmt(self, f)
    }
}

impl std::error::Error for MsgConversionError {}

#[cfg(feature = "derive")]
fn system_endian() -> Endianness {
    if cfg!(target_endian = "big") {
        Endianness::Big
    } else if cfg!(target_endian = "little") {
        Endianness::Little
    } else {
        panic!("Unsupported endianness");
    }
}

#[derive(Clone, Debug)]
pub struct PointCloud2Msg {
    pub header: HeaderMsg,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointFieldMsg>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}

impl Default for PointCloud2Msg {
    fn default() -> Self {
        Self {
            header: HeaderMsg::default(),
            height: 1, // everything is in one row (unstructured)
            width: 0,
            fields: Vec::new(),
            is_bigendian: false, // ROS default
            point_step: 0,
            row_step: 0,
            data: Vec::new(),
            is_dense: false, // ROS default
        }
    }
}

impl PointCloud2Msg {
    #[cfg(feature = "derive")]
    fn prepare_direct_copy<const N: usize, C>() -> Result<Self, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        let point: RPCL2Point<N> = C::default().into();
        debug_assert!(point.fields.len() == N);

        let meta_names = C::field_names_ordered();
        debug_assert!(meta_names.len() == N);

        let mut offset: u32 = 0;
        let layout = TypeLayoutInfo::try_from(C::type_layout())?;
        let mut fields: Vec<PointFieldMsg> = Vec::with_capacity(layout.fields.len());
        for f in layout.fields.into_iter() {
            match f {
                PointField::Field {
                    name,
                    datatype,
                    size,
                } => {
                    fields.push(PointFieldMsg {
                        name,
                        offset,
                        datatype,
                        ..Default::default()
                    });
                    offset += size; // assume field_count 1
                }
                PointField::Padding(size) => {
                    offset += size; // assume field_count 1
                }
            }
        }

        Ok(PointCloud2Msg {
            point_step: offset,
            fields,
            ..Default::default()
        })
    }

    #[cfg(feature = "derive")]
    fn assert_byte_similarity<const N: usize, C>(&self) -> Result<bool, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        let point: RPCL2Point<N> = C::default().into();
        debug_assert!(point.fields.len() == N);

        let meta_names = C::field_names_ordered();
        debug_assert!(meta_names.len() == N);

        let mut offset: u32 = 0;
        let layout = TypeLayoutInfo::try_from(C::type_layout())?;
        for (f, msg_f) in layout.fields.into_iter().zip(self.fields.iter()) {
            match f {
                PointField::Field {
                    name,
                    datatype,
                    size,
                } => {
                    if msg_f.name != name {
                        return Err(MsgConversionError::FieldNotFound(vec![name.clone()]));
                    }

                    if msg_f.datatype != datatype {
                        return Err(MsgConversionError::InvalidFieldFormat);
                    }

                    if msg_f.offset != offset {
                        return Err(MsgConversionError::DataLengthMismatch);
                    }

                    if msg_f.count != 1 {
                        return Err(MsgConversionError::DataLengthMismatch);
                    }

                    offset += size; // assume field_count 1
                }
                PointField::Padding(size) => {
                    offset += size; // assume field_count 1
                }
            }
        }

        Ok(true)
    }

    #[inline(always)]
    fn prepare<const N: usize, C>() -> Result<Self, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        let point: RPCL2Point<N> = C::default().into();
        debug_assert!(point.fields.len() == N);

        let meta_names = C::field_names_ordered();
        debug_assert!(meta_names.len() == N);

        let mut meta_offsets_acc = 0;
        let mut fields = vec![PointFieldMsg::default(); N];
        for ((meta_value, meta_name), field_val) in point
            .fields
            .into_iter()
            .zip(meta_names.into_iter())
            .zip(fields.iter_mut())
        {
            let datatype_code = meta_value.datatype.into();
            FieldDatatype::try_from(datatype_code)?;

            let field_count = 1;

            *field_val = PointFieldMsg {
                name: meta_name.into(),
                offset: meta_offsets_acc,
                datatype: datatype_code,
                count: 1,
            };

            meta_offsets_acc += field_count * meta_value.datatype.size() as u32
        }

        Ok(PointCloud2Msg {
            point_step: meta_offsets_acc,
            fields,
            ..Default::default()
        })
    }

    /// Create a PointCloud2Msg from any iterable type.
    ///
    /// The operation is O(n) in time complexity where n is the number of points in the point cloud.
    ///
    /// # Example
    /// ```
    /// use ros_pointcloud2::prelude::*;
    ///
    /// let cloud_points: Vec<PointXYZ> = vec![
    ///     PointXYZ::new(1.0, 2.0, 3.0),
    ///     PointXYZ::new(4.0, 5.0, 6.0),
    /// ];
    ///
    // let msg_out = PointCloud2Msg::try_from_iter(cloud_points).unwrap();
    /// ```
    pub fn try_from_iter<const N: usize, C>(
        iterable: impl IntoIterator<Item = C>,
    ) -> Result<Self, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        let mut cloud = Self::prepare::<N, C>()?;

        iterable.into_iter().for_each(|coords| {
            let point: RPCL2Point<N> = coords.into();

            point.fields.iter().for_each(|meta| {
                let truncated_bytes = unsafe {
                    std::slice::from_raw_parts(meta.bytes.as_ptr(), meta.datatype.size())
                };
                cloud.data.extend_from_slice(truncated_bytes);
            });

            cloud.width += 1;
        });

        cloud.row_step = cloud.width * cloud.point_step;

        Ok(cloud)
    }

    #[cfg(feature = "derive")]
    pub fn try_from_vec<const N: usize, C>(vec: Vec<C>) -> Result<Self, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        match system_endian() {
            Endianness::Big => Self::try_from_iter(vec.into_iter()),
            Endianness::Little => {
                let mut cloud = Self::prepare_direct_copy::<N, C>()?;

                let bytes_total = vec.len() * cloud.point_step as usize;
                cloud.data.resize(bytes_total, u8::default());
                let raw_data: *mut C = cloud.data.as_ptr() as *mut C;
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        vec.as_ptr() as *const u8,
                        raw_data as *mut u8,
                        bytes_total,
                    );
                }

                cloud.width = vec.len() as u32;
                cloud.row_step = cloud.width * cloud.point_step;

                Ok(cloud)
            }
        }
    }

    #[cfg(feature = "derive")]
    pub fn try_into_vec<const N: usize, C>(self) -> Result<Vec<C>, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        self.assert_byte_similarity::<N, C>()?;

        match system_endian() {
            Endianness::Big => Ok(self.try_into_iter()?.collect()),
            Endianness::Little => {
                let mut vec = Vec::with_capacity(self.width as usize);
                let raw_data: *const C = self.data.as_ptr() as *const C;
                unsafe {
                    for i in 0..self.width {
                        let point = raw_data.add(i as usize).read();
                        vec.push(point);
                    }
                }

                Ok(vec)
            }
        }
    }

    pub fn try_into_iter<const N: usize, C>(
        self,
    ) -> Result<impl Iterator<Item = C>, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        iterator::PointCloudIterator::try_from(self)
    }

    #[cfg(feature = "rayon")]
    pub fn try_into_par_iter<const N: usize, C>(
        self,
    ) -> Result<impl rayon::iter::ParallelIterator<Item = C>, MsgConversionError>
    where
        C: PointConvertible<N> + Send + Sync,
    {
        iterator::PointCloudIterator::try_from(self)
    }
}

/// Internal point representation. It is used to store the coordinates and meta data of a point.
/// In each iteration, an internal point representation is converted to the desired point type.
/// Implement the `From` traits for your point type to use the conversion.
///
/// See the [`ros_pointcloud2::PointConvertible`] trait for more information.
pub struct RPCL2Point<const N: usize> {
    pub fields: [PointMeta; N],
}

impl<const N: usize> Default for RPCL2Point<N> {
    fn default() -> Self {
        Self {
            fields: [PointMeta::default(); N],
        }
    }
}

impl<const N: usize> From<[PointMeta; N]> for RPCL2Point<N> {
    fn from(fields: [PointMeta; N]) -> Self {
        Self { fields }
    }
}

/// Trait to enable point conversions on the fly while iterating.
///
/// Implement this trait for your custom point you want to read or write in the message.
///
/// # Example
/// ```
/// use ros_pointcloud2::prelude::*;
///
/// #[derive(Clone, Debug, PartialEq, Copy, Default)]
/// pub struct MyPointXYZI {
///     pub x: f32,
///     pub y: f32,
///     pub z: f32,
///     pub intensity: f32,
/// }
///
/// impl From<MyPointXYZI> for RPCL2Point<4> {
///     fn from(point: MyPointXYZI) -> Self {
///         RPCL2Point {
///             fields: [point.x.into(), point.y.into(), point.z.into(), point.intensity.into()],
///         }
///     }
/// }
///
/// impl From<RPCL2Point<4>> for MyPointXYZI {
///     fn from(point: RPCL2Point<4>) -> Self {
///         Self {
///             x: point.fields[0].get(),
///             y: point.fields[1].get(),
///             z: point.fields[2].get(),
///             intensity: point.fields[3].get(),
///         }
///     }
/// }
///
/// impl Fields<4> for MyPointXYZI {
///    fn field_names_ordered() -> [&'static str; 4] {
///       ["x", "y", "z", "intensity"]
///   }
/// }
///
/// impl PointConvertible<4> for MyPointXYZI {}
/// ```
#[cfg(not(feature = "derive"))]
pub trait PointConvertible<const N: usize>:
    From<RPCL2Point<N>> + Into<RPCL2Point<N>> + Fields<N> + Clone + 'static + Default
{
}

#[cfg(feature = "derive")]
pub trait PointConvertible<const N: usize>:
    type_layout::TypeLayout + From<RPCL2Point<N>> + Into<RPCL2Point<N>> + Fields<N> + 'static + Default
{
}

#[cfg(feature = "derive")]
enum PointField {
    Padding(u32),
    Field {
        name: String,
        size: u32,
        datatype: u8,
    },
}

#[cfg(feature = "derive")]
struct TypeLayoutInfo {
    fields: Vec<PointField>,
}

#[cfg(feature = "derive")]
impl TryFrom<type_layout::Field> for PointField {
    type Error = MsgConversionError;

    fn try_from(f: type_layout::Field) -> Result<Self, Self::Error> {
        match f {
            type_layout::Field::Field { name, ty, size } => {
                let typename: String = ty.to_owned().into();
                let datatype = FieldDatatype::try_from(typename)?;
                Ok(Self::Field {
                    name: name.to_owned().into(),
                    size: size as u32,
                    datatype: datatype.into(),
                })
            }
            type_layout::Field::Padding { size } => Ok(Self::Padding(size as u32)),
        }
    }
}

#[cfg(feature = "derive")]
impl TryFrom<type_layout::TypeLayoutInfo> for TypeLayoutInfo {
    type Error = MsgConversionError;

    fn try_from(t: type_layout::TypeLayoutInfo) -> Result<Self, Self::Error> {
        let fields: Vec<PointField> = t
            .fields
            .into_iter()
            .map(PointField::try_from)
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self { fields })
    }
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
    endianness: Endianness,
    datatype: FieldDatatype,
}

impl Default for PointMeta {
    fn default() -> Self {
        Self {
            bytes: [u8::default(); std::mem::size_of::<f64>()],
            datatype: FieldDatatype::F32,
            endianness: Endianness::default(),
        }
    }
}

impl PointMeta {
    /// Create a new PointMeta from a value.
    ///
    /// # Example
    /// ```
    /// let meta = ros_pointcloud2::PointMeta::new(1.0);
    /// ```
    #[inline(always)]
    pub fn new<T: FromBytes>(value: T) -> Self {
        let raw_bytes = T::bytes(&value);
        let mut bytes = [0; std::mem::size_of::<f64>()];
        for (byte, save_byte) in raw_bytes.into_iter().zip(bytes.iter_mut()) {
            *save_byte = byte;
        }

        Self {
            bytes,
            datatype: T::field_datatype(),
            ..Default::default()
        }
    }

    #[inline(always)]
    fn from_buffer(
        data: &[u8],
        offset: usize,
        datatype: FieldDatatype,
        endianness: Endianness,
    ) -> Self {
        debug_assert!(data.len() >= offset + datatype.size());
        let bytes = [u8::default(); std::mem::size_of::<f64>()];
        unsafe {
            let data_ptr = data.as_ptr().add(offset);
            let bytes_ptr = bytes.as_ptr() as *mut u8;
            std::ptr::copy_nonoverlapping(data_ptr, bytes_ptr, datatype.size());
        }

        Self {
            bytes,
            datatype,
            endianness,
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

        match self.endianness {
            Endianness::Big => T::from_be_bytes(bytes),
            Endianness::Little => T::from_le_bytes(bytes),
        }
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
