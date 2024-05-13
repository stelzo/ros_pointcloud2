//! A PointCloud2 message conversion library.
//!
//! The library provides the [`ros_pointcloud2::PointCloud2Msg`] type, which implements conversions to and from `Vec` and (parallel) iterators.
//!
//! - [`ros_pointcloud2::PointCloud2Msg::try_from_vec`]
//! - [`ros_pointcloud2::PointCloud2Msg::try_into_vec`]
//! - [`ros_pointcloud2::PointCloud2Msg::try_from_iter`]
//! - [`ros_pointcloud2::PointCloud2Msg::try_into_iter`]
//! - [`ros_pointcloud2::PointCloud2Msg::try_into_par_iter`]
//! - [`ros_pointcloud2::PointCloud2Msg::try_from_par_iter`]
//!
//! The best choice depends on your use case and the point cloud size.
//! A good rule of thumb for minimum latency is to use `_vec` per default.
//!
//! The `_iter` APIs bring more predictable performance and avoid memory allocation but are slower in general (see Performance section in the repository).
//! If you do any processing on larger point clouds or heavy processing on any sized cloud, switch to `_par_iter`.
//!
//! For ROS interoperability, there are implementations for the `r2r`, `rclrs` (ros2_rust) and `rosrust` message types
//! available with feature flags. If you miss a message type, please open an issue or a PR.
//! See the [`ros_pointcloud2::ros_types`] module for more information.
//!
//! Common point types like [`ros_pointcloud2::points::PointXYZ`] or
//! [`ros_pointcloud2::points::PointXYZI`] are provided. You can easily add any additional custom type.
//! See `examples/custom_enum_field_filter.rs` for an example.
//!
//! # Example PointXYZ
//! ```
//! use ros_pointcloud2::prelude::*;
//!
//! let cloud_points = vec![
//!   PointXYZ::new(9.0006, 42.0, -6.2),
//!   PointXYZ::new(f32::MAX, f32::MIN, f32::MAX),
//! ];
//! let cloud_copy = cloud_points.clone(); // For equality test later.
//!
//! let in_msg = PointCloud2Msg::try_from_vec(cloud_points).unwrap();
//!
//! // Convert to your ROS crate message type, we will use r2r here.
//! // let msg: r2r::sensor_msgs::msg::PointCloud2 = in_msg.into();
//! // Publish ...
//! // ... now incoming from a topic.
//! // let in_msg: PointCloud2Msg = msg.into();
//!
//! let new_pcl = in_msg.try_into_iter().unwrap()
//!     .map(|point: PointXYZ| { // Define the data you want from the point.
//!         // Some logic here.
//!
//!         point
//!     })
//!     .collect::<Vec<_>>();
//! assert_eq!(new_pcl, cloud_copy);
//! ```
//!
//! # Features
//! In addition to the ROS intregrations, the following features are available:
//! - 'derive' (default): Helpful derive macros for custom point types. Also, derive enables direct copy with `_vec` endpoints.
//! - 'rayon': Parallel iterator support for `_par_iter` functions. `try_from_par_iter` additionally needs the 'derive' feature to be enabled.
//! - 'nalgebra': Predefined points offer `xyz()` getter functions for `nalgebra::Point3` types.

pub mod convert;
pub mod points;
pub mod prelude;
pub mod ros_types;

pub mod iterator;

use std::num::TryFromIntError;
use std::str::FromStr;

use crate::convert::{FieldDatatype, FromBytes};
use crate::ros_types::{HeaderMsg, PointFieldMsg};
pub use convert::Fields;
use convert::{ByteSimilarity, Denseness, Endian};

/// All errors that can occur while converting to or from the message type.
#[derive(Debug)]
pub enum MsgConversionError {
    InvalidFieldFormat,
    UnsupportedFieldType(String),
    DataLengthMismatch,
    FieldsNotFound(Vec<String>),
    UnsupportedFieldCount,
    NumberConversion,
}

impl From<TryFromIntError> for MsgConversionError {
    fn from(_: TryFromIntError) -> Self {
        MsgConversionError::NumberConversion
    }
}

impl std::fmt::Display for MsgConversionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MsgConversionError::InvalidFieldFormat => {
                write!(f, "The field does not match the expected datatype.")
            }
            MsgConversionError::UnsupportedFieldType(datatype) => {
                write!(
                    f,
                    "The field datatype is not supported by the ROS message description: {}",
                    datatype
                )
            }
            MsgConversionError::DataLengthMismatch => {
                write!(f, "The length of the byte buffer in the message does not match the expected length computed from the fields, indicating a corrupted or malformed message.")
            }
            MsgConversionError::FieldsNotFound(fields) => {
                write!(f, "Some fields are not found in the message: {:?}", fields)
            }
            MsgConversionError::UnsupportedFieldCount => {
                write!(
                    f,
                    "Only field_count 1 is supported for reading and writing."
                )
            }
            MsgConversionError::NumberConversion => {
                write!(f, "The number is too large to be converted into a PointCloud2 supported datatype.")
            }
        }
    }
}

impl std::error::Error for MsgConversionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        None
    }
}

#[cfg(feature = "derive")]
fn system_endian() -> Endian {
    if cfg!(target_endian = "big") {
        Endian::Big
    } else if cfg!(target_endian = "little") {
        Endian::Little
    } else {
        panic!("Unsupported Endian");
    }
}

/// A PointCloud2 message type.
///
/// This representation is a small abstraction of the ROS message description, since every ROS library generates its own messages.
/// To assert consistency, the type should be build with the [`ros_pointcloud2::PointCloud2MsgBuilder`].
#[derive(Clone, Debug)]
pub struct PointCloud2Msg {
    pub header: HeaderMsg,
    pub dimensions: CloudDimensions,
    pub fields: Vec<PointFieldMsg>,
    pub endian: Endian,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub dense: Denseness,
}

#[derive(Clone, Debug)]
pub struct CloudDimensionsBuilder(usize);

impl CloudDimensionsBuilder {
    pub fn new_with_width(width: usize) -> Self {
        Self(width)
    }

    pub fn build(self) -> Result<CloudDimensions, MsgConversionError> {
        let width = match u32::try_from(self.0) {
            Ok(w) => w,
            Err(_) => return Err(MsgConversionError::NumberConversion),
        };

        Ok(CloudDimensions {
            width,
            height: if self.0 > 0 { 1 } else { 0 },
        })
    }
}

/// Creating a PointCloud2Msg with a builder pattern to avoid invalid states.
#[derive(Clone, Debug, Default)]
pub struct PointCloud2MsgBuilder {
    header: HeaderMsg,
    width: u32,
    fields: Vec<PointFieldMsg>,
    is_big_endian: bool,
    point_step: u32,
    row_step: u32,
    data: Vec<u8>,
    is_dense: bool,
}

impl PointCloud2MsgBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn header(mut self, header: HeaderMsg) -> Self {
        self.header = header;
        self
    }

    pub fn width(mut self, width: u32) -> Self {
        self.width = width;
        self
    }

    pub fn fields(mut self, fields: Vec<PointFieldMsg>) -> Self {
        self.fields = fields;
        self
    }

    pub fn endian(mut self, is_big_endian: bool) -> Self {
        self.is_big_endian = is_big_endian;
        self
    }

    pub fn point_step(mut self, point_step: u32) -> Self {
        self.point_step = point_step;
        self
    }

    pub fn row_step(mut self, row_step: u32) -> Self {
        self.row_step = row_step;
        self
    }

    pub fn data(mut self, data: Vec<u8>) -> Self {
        self.data = data;
        self
    }

    pub fn dense(mut self, is_dense: bool) -> Self {
        self.is_dense = is_dense;
        self
    }

    pub fn build(self) -> Result<PointCloud2Msg, MsgConversionError> {
        if self.fields.is_empty() {
            return Err(MsgConversionError::FieldsNotFound(vec![]));
        }

        if self.fields.iter().any(|f| f.count != 1) {
            return Err(MsgConversionError::UnsupportedFieldCount);
        }

        let fields_size = self
            .fields
            .iter()
            .map(FieldDatatype::try_from)
            .collect::<Result<Vec<_>, _>>()?
            .iter()
            .map(|f| f.size() as u32)
            .sum::<_>();

        if self.point_step < fields_size {
            return Err(MsgConversionError::InvalidFieldFormat);
        }

        if self.data.len() as u32 % self.point_step != 0 {
            return Err(MsgConversionError::DataLengthMismatch);
        }

        Ok(PointCloud2Msg {
            header: self.header,
            dimensions: CloudDimensionsBuilder::new_with_width(self.width as usize).build()?,
            fields: self.fields,
            endian: if self.is_big_endian {
                Endian::Big
            } else {
                Endian::Little
            },
            point_step: self.point_step,
            row_step: self.row_step,
            data: self.data,
            dense: if self.is_dense {
                Denseness::Dense
            } else {
                Denseness::Sparse
            },
        })
    }
}

#[derive(Clone, Debug, Default)]
pub struct CloudDimensions {
    pub width: u32,
    pub height: u32,
}

impl PointCloud2Msg {
    #[cfg(feature = "derive")]
    fn prepare_direct_copy<const N: usize, C>() -> Result<PointCloud2MsgBuilder, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        let point: RPCL2Point<N> = C::default().into();
        debug_assert!(point.fields.len() == N);

        let field_names = C::field_names_ordered();
        debug_assert!(field_names.len() == N);

        let layout = TypeLayoutInfo::try_from(C::type_layout())?;
        debug_assert!(field_names.len() == layout.fields.len());

        let mut offset = 0;
        let mut fields: Vec<PointFieldMsg> = Vec::with_capacity(layout.fields.len());
        for f in layout.fields.into_iter() {
            let f_translated = field_names[fields.len()].to_string();
            match f {
                PointField::Field {
                    datatype,
                    size,
                    count,
                } => {
                    fields.push(PointFieldMsg {
                        name: f_translated,
                        offset,
                        datatype,
                        ..Default::default()
                    });
                    offset += size * count;
                }
                PointField::Padding(size) => {
                    offset += size;
                }
            }
        }

        Ok(PointCloud2MsgBuilder::new()
            .fields(fields)
            .point_step(offset))
    }

    #[cfg(feature = "derive")]
    fn assert_byte_similarity<const N: usize, C>(
        &self,
    ) -> Result<ByteSimilarity, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        let point: RPCL2Point<N> = C::default().into();
        debug_assert!(point.fields.len() == N);

        let field_names = C::field_names_ordered();
        debug_assert!(field_names.len() == N);

        let layout = TypeLayoutInfo::try_from(C::type_layout())?;
        debug_assert!(field_names.len() == layout.fields.len());

        let mut offset: u32 = 0;
        let mut field_counter = 0;
        for (f, msg_f) in layout.fields.iter().zip(self.fields.iter()) {
            match f {
                PointField::Field {
                    datatype,
                    size,
                    count,
                } => {
                    let f_translated = field_names[field_counter].to_string();
                    field_counter += 1;

                    if msg_f.name != f_translated
                        || msg_f.offset != offset
                        || msg_f.datatype != *datatype
                        || msg_f.count != 1
                    {
                        return Ok(ByteSimilarity::Different);
                    }

                    offset += size * count;
                }
                PointField::Padding(size) => {
                    offset += size;
                }
            }
        }

        Ok(if offset == self.point_step {
            ByteSimilarity::Equal
        } else {
            ByteSimilarity::Overlapping
        })
    }

    #[inline(always)]
    fn prepare<const N: usize, C>() -> Result<PointCloud2MsgBuilder, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        let point: RPCL2Point<N> = C::default().into();
        debug_assert!(point.fields.len() == N);

        let field_names = C::field_names_ordered();
        debug_assert!(field_names.len() == N);

        let mut meta_offsets_acc: u32 = 0;
        let mut fields = vec![PointFieldMsg::default(); N];
        let field_count: u32 = 1;
        for ((meta_value, field_name), field_val) in point
            .fields
            .into_iter()
            .zip(field_names.into_iter())
            .zip(fields.iter_mut())
        {
            let datatype_code = meta_value.datatype.into();
            let _ = FieldDatatype::try_from(datatype_code)?;

            *field_val = PointFieldMsg {
                name: field_name.into(),
                offset: meta_offsets_acc,
                datatype: datatype_code,
                count: 1,
            };

            meta_offsets_acc += field_count * meta_value.datatype.size() as u32;
        }

        Ok(PointCloud2MsgBuilder::new()
            .fields(fields)
            .point_step(meta_offsets_acc))
    }

    /// Create a PointCloud2Msg from any iterable type.
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
        let mut cloud_width = 0;
        let cloud_point_step = cloud.point_step;

        iterable.into_iter().for_each(|pointdata| {
            let point: RPCL2Point<N> = pointdata.into();

            point.fields.iter().for_each(|meta| {
                let truncated_bytes = unsafe {
                    std::slice::from_raw_parts(meta.bytes.as_ptr(), meta.datatype.size())
                };
                cloud.data.extend_from_slice(truncated_bytes);
            });

            cloud_width += 1;
        });

        cloud = cloud.width(cloud_width);
        cloud = cloud.row_step(cloud_width * cloud_point_step);

        cloud.build()
    }

    /// Create a PointCloud2Msg from a parallel iterator. Requires the `rayon` and `derive` feature to be enabled.
    #[cfg(all(feature = "rayon", feature = "derive"))]
    pub fn try_from_par_iter<const N: usize, C>(
        iterable: impl rayon::iter::ParallelIterator<Item = C>,
    ) -> Result<Self, MsgConversionError>
    where
        C: PointConvertible<N> + Send + Sync,
    {
        Self::try_from_vec(iterable.collect::<Vec<_>>())
    }

    /// Create a PointCloud2Msg from a Vec of points.
    /// Since the point type is known at compile time, the conversion is done by direct copy.
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
    /// let msg_out = PointCloud2Msg::try_from_vec(cloud_points).unwrap();
    /// ```
    #[cfg(feature = "derive")]
    pub fn try_from_vec<const N: usize, C>(vec: Vec<C>) -> Result<Self, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        match (system_endian(), Endian::default()) {
            (Endian::Big, Endian::Big) | (Endian::Little, Endian::Little) => {
                let mut cloud = Self::prepare_direct_copy::<N, C>()?;
                let point_step = cloud.point_step;

                let bytes_total = vec.len() * point_step as usize;
                cloud.data.resize(bytes_total, u8::default());
                let raw_data: *mut C = cloud.data.as_ptr() as *mut C;
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        vec.as_ptr() as *const u8,
                        raw_data as *mut u8,
                        bytes_total,
                    );
                }

                Ok(cloud
                    .width(vec.len() as u32)
                    .row_step(vec.len() as u32 * point_step)
                    .build()?)
            }
            _ => Self::try_from_iter(vec),
        }
    }

    /// Convert the PointCloud2Msg to a Vec of points.
    ///
    /// # Example
    /// ```
    /// use ros_pointcloud2::prelude::*;
    ///
    /// let cloud_points: Vec<PointXYZI> = vec![
    ///     PointXYZI::new(1.0, 2.0, 3.0, 0.5),
    ///     PointXYZI::new(4.0, 5.0, 6.0, 1.1),
    /// ];
    ///
    /// let msg_out = PointCloud2Msg::try_from_vec(cloud_points).unwrap();
    /// let cloud_points_out: Vec<PointXYZ> = msg_out.try_into_vec().unwrap();
    /// assert_eq!(1.0, cloud_points_out.get(0).unwrap().x);
    /// ```
    #[cfg(feature = "derive")]
    pub fn try_into_vec<const N: usize, C>(self) -> Result<Vec<C>, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        match (system_endian(), self.endian) {
            (Endian::Big, Endian::Big) | (Endian::Little, Endian::Little) => {
                let bytematch = match self.assert_byte_similarity::<N, C>()? {
                    ByteSimilarity::Equal => true,
                    ByteSimilarity::Overlapping => false,
                    ByteSimilarity::Different => return Ok(self.try_into_iter()?.collect()),
                };

                let cloud_width = self.dimensions.width as usize;
                let point_step = self.point_step as usize;
                let mut vec = Vec::with_capacity(cloud_width);
                if bytematch {
                    unsafe {
                        std::ptr::copy_nonoverlapping(
                            self.data.as_ptr(),
                            vec.as_mut_ptr() as *mut u8,
                            self.data.len(),
                        );
                    }
                } else {
                    unsafe {
                        for i in 0..cloud_width {
                            let point_ptr = self.data.as_ptr().add(i * point_step) as *const C;
                            let point = point_ptr.read();
                            vec.push(point);
                        }
                    }
                }

                Ok(vec)
            }
            _ => Ok(self.try_into_iter()?.collect()), // Endianess does not match, read point by point since Endian is read at conversion time.
        }
    }

    /// Convert the PointCloud2Msg to an iterator.
    ///
    /// # Example
    /// ```
    /// use ros_pointcloud2::prelude::*;
    ///
    /// let cloud_points: Vec<PointXYZI> = vec![
    ///     PointXYZI::new(1.0, 2.0, 3.0, 0.5),
    ///     PointXYZI::new(4.0, 5.0, 6.0, 1.1),
    /// ];
    ///
    /// let msg_out = PointCloud2Msg::try_from_iter(cloud_points).unwrap();
    /// let cloud_points_out = msg_out.try_into_iter().unwrap().collect::<Vec<PointXYZ>>();
    /// ```
    pub fn try_into_iter<const N: usize, C>(
        self,
    ) -> Result<impl Iterator<Item = C>, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        iterator::PointCloudIterator::try_from(self)
    }

    /// Convert the PointCloud2Msg to a parallel iterator. Requires the `rayon` feature to be enabled.
    ///
    /// # Example
    /// ```
    /// use ros_pointcloud2::prelude::*;
    ///
    /// let cloud_points: Vec<PointXYZI> = vec![
    ///    PointXYZI::new(1.0, 2.0, 3.0, 0.5),
    ///    PointXYZI::new(4.0, 5.0, 6.0, 1.1),
    /// ];
    ///
    /// let msg_out = PointCloud2Msg::try_from_iter(cloud_points).unwrap();
    /// let cloud_points_out = msg_out.try_into_par_iter().unwrap().collect::<Vec<PointXYZ>>();
    /// assert_eq!(2, cloud_points_out.len());
    /// ```
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
    fields: [PointData; N],
}

impl<const N: usize> Default for RPCL2Point<N> {
    fn default() -> Self {
        Self {
            fields: [PointData::default(); N],
        }
    }
}

impl<const N: usize> std::ops::Index<usize> for RPCL2Point<N> {
    type Output = PointData;

    fn index(&self, index: usize) -> &Self::Output {
        &self.fields[index]
    }
}

impl<const N: usize> From<[PointData; N]> for RPCL2Point<N> {
    fn from(fields: [PointData; N]) -> Self {
        Self { fields }
    }
}

/// Trait to enable point conversions on the fly.
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
///         [point.x.into(), point.y.into(), point.z.into(), point.intensity.into()].into()
///     }
/// }
///
/// impl From<RPCL2Point<4>> for MyPointXYZI {
///     fn from(point: RPCL2Point<4>) -> Self {
///         Self {
///             x: point[0].get(),
///             y: point[1].get(),
///             z: point[2].get(),
///             intensity: point[3].get(),
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

/// Trait to enable point conversions on the fly.
///
/// Implement this trait for your custom point you want to read or write in the message.
/// For a more convenient way to implement this trait, enable the `derive` feature and use the `#[derive(PointConvertible, TypeLayout)]` macro.
///
/// # Derive Example
/// ```
/// use ros_pointcloud2::prelude::*;
///
/// #[derive(Clone, Debug, PartialEq, Copy, Default, PointConvertible, TypeLayout)]
/// pub struct MyPointXYZI {
///     pub x: f32,
///     pub y: f32,
///     pub z: f32,
///     pub intensity: f32,
/// }
/// ```
///
/// # Manual Example
/// ```
/// use ros_pointcloud2::prelude::*;
///
/// #[derive(Clone, Debug, PartialEq, Copy, Default, TypeLayout)]
/// pub struct MyPointXYZI {
///     pub x: f32,
///     pub y: f32,
///     pub z: f32,
///     pub intensity: f32,
/// }
///
/// impl From<MyPointXYZI> for RPCL2Point<4> {
///     fn from(point: MyPointXYZI) -> Self {
///         [point.x.into(), point.y.into(), point.z.into(), point.intensity.into()].into()
///     }
/// }
///
/// impl From<RPCL2Point<4>> for MyPointXYZI {
///     fn from(point: RPCL2Point<4>) -> Self {
///         Self {
///             x: point[0].get(),
///             y: point[1].get(),
///             z: point[2].get(),
///             intensity: point[3].get(),
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
#[cfg(feature = "derive")]
pub trait PointConvertible<const N: usize>:
    type_layout::TypeLayout + From<RPCL2Point<N>> + Into<RPCL2Point<N>> + Fields<N> + 'static + Default
{
}

#[cfg(feature = "derive")]
enum PointField {
    Padding(u32),
    Field { size: u32, datatype: u8, count: u32 },
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
            type_layout::Field::Field { name: _, ty, size } => {
                let typename: String = ty.into_owned().to_lowercase();
                let datatype = FieldDatatype::from_str(typename.as_str())?;
                Ok(Self::Field {
                    size: size.try_into()?,
                    datatype: datatype.into(),
                    count: 1,
                })
            }
            type_layout::Field::Padding { size } => Ok(Self::Padding(size.try_into()?)),
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

/// Single data representation for a point.
///
/// This struct is used to store data fields in a fixed size byte buffer along the with the
/// datatype that is encoded so that it can be decoded later.
///
/// # Example
/// ```
/// use ros_pointcloud2::PointData;
///
/// let original_data: f64 = 1.0;
/// let meta = PointData::new(original_data);
/// let my_data: f64 = meta.get();
/// ```
#[derive(Debug, Clone, Copy)]
pub struct PointData {
    bytes: [u8; std::mem::size_of::<f64>()],
    endian: Endian,
    datatype: FieldDatatype,
}

impl Default for PointData {
    fn default() -> Self {
        Self {
            bytes: [u8::default(); std::mem::size_of::<f64>()],
            datatype: FieldDatatype::F32,
            endian: Endian::default(),
        }
    }
}

impl PointData {
    /// Create a new PointData from a value.
    ///
    /// # Example
    /// ```
    /// let meta = ros_pointcloud2::PointData::new(1.0);
    /// ```
    #[inline(always)]
    pub fn new<T: FromBytes>(value: T) -> Self {
        Self {
            bytes: value.into().raw(),
            datatype: T::field_datatype(),
            ..Default::default()
        }
    }

    #[inline(always)]
    fn from_buffer(data: &[u8], offset: usize, datatype: FieldDatatype, endian: Endian) -> Self {
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
            endian,
        }
    }

    /// Get the numeric value from the PointData description.
    ///
    /// # Example
    /// ```
    /// let original_data: f64 = 1.0;
    /// let meta = ros_pointcloud2::PointData::new(original_data);
    /// let my_data: f64 = meta.get();
    /// ```
    pub fn get<T: FromBytes>(&self) -> T {
        match self.endian {
            Endian::Big => T::from_be_bytes(convert::PointDataBuffer::new(self.bytes)),
            Endian::Little => T::from_le_bytes(convert::PointDataBuffer::new(self.bytes)),
        }
    }
}

impl From<f32> for PointData {
    fn from(value: f32) -> Self {
        Self::new(value)
    }
}

impl From<f64> for PointData {
    fn from(value: f64) -> Self {
        Self::new(value)
    }
}

impl From<i32> for PointData {
    fn from(value: i32) -> Self {
        Self::new(value)
    }
}

impl From<u8> for PointData {
    fn from(value: u8) -> Self {
        Self::new(value)
    }
}

impl From<u16> for PointData {
    fn from(value: u16) -> Self {
        Self::new(value)
    }
}

impl From<u32> for PointData {
    fn from(value: u32) -> Self {
        Self::new(value)
    }
}

impl From<i8> for PointData {
    fn from(value: i8) -> Self {
        Self::new(value)
    }
}

impl From<i16> for PointData {
    fn from(value: i16) -> Self {
        Self::new(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rpcl2_derive::Fields;

    #[allow(dead_code)]
    #[derive(Fields)]
    struct TestStruct {
        field1: String,
        #[rpcl2(name = "renamed_field")]
        field2: i32,
        field3: f64,
        field4: bool,
    }

    #[test]
    fn test_struct_names() {
        let names = TestStruct::field_names_ordered();
        assert_eq!(names, ["field1", "renamed_field", "field3", "field4"]);
    }
}
