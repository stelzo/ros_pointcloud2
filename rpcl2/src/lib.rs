//! A PointCloud2 message conversion library.
//!
//! The library provides the [`PointCloud2Msg`] type, which implements conversions to and from `Vec` and (parallel) iterators.
//!
//! Vector conversions are optimized for direct copy. They are useful when you just move similar data around. They are usually a good default.
//! - [`try_from_vec`](PointCloud2Msg::try_from_vec)
//! - [`try_into_vec`](PointCloud2Msg::try_into_vec)
//!
//! You can use the iterator functions for more control over the conversion process.
//! - [`try_from_iter`](PointCloud2Msg::try_from_iter)
//! - [`try_into_iter`](PointCloud2Msg::try_into_iter)
//!
//! These feature predictable performance but they do not scale well with large clouds. Learn more about that in the [performance section](https://github.com/stelzo/ros_pointcloud2?tab=readme-ov-file#performance) of the repository.
//! The iterators are useful when your conversions are more complex than a simple copy or the cloud is small enough.
//!
//! When the cloud is getting larger or you are doing a lot of processing per point, switch to the parallel iterators.
//! - [`try_into_par_iter`](PointCloud2Msg::try_into_par_iter) requires `rayon` feature
//! - [`try_from_par_iter`](PointCloud2Msg::try_from_par_iter) requires `rayon` feature
//!
//! For ROS interoperability, there are integrations available with feature flags. If you miss a message type, please open an issue or a PR.
//! See the [`ros`] module for more information on how to integrate more libraries.
//!
//! Common point types like [`PointXYZ`](points::PointXYZ) or [`PointXYZI`](points::PointXYZI) are provided. See the full list [`here`](points). You can easily add any additional custom type.
//! See [custom_enum_field_filter.rs](https://github.com/stelzo/ros_pointcloud2/blob/main/rpcl2/examples/custom_enum_field_filter.rs) for an example.
//!
//! # Minimal Example
//! ```
//! use ros_pointcloud2::prelude::*;
//!
//! let cloud_points = vec![
//!     PointXYZI::new(9.6, 42.0, -6.2, 0.1),
//!     PointXYZI::new(46.0, 5.47, 0.5, 0.1),
//! ];
//! let cloud_copy = cloud_points.clone(); // For equality test later.
//!
//! let out_msg = PointCloud2Msg::try_from_iter(cloud_points).unwrap();
//!
//! // Convert to your ROS crate message type.
//! // let msg: r2r::sensor_msgs::msg::PointCloud2 = in_msg.into();
//! // Publish ...
//!
//! // ... now incoming from a topic.
//! // let in_msg: PointCloud2Msg = msg.into();
//! let in_msg = out_msg;
//!
//! let processed_cloud = in_msg.try_into_iter().unwrap()
//!     .map(|point: PointXYZ| { // Define the data you want from the point.
//!         // Some logic here.
//!         PointXYZI::new(point.x, point.y, point.z, 0.1)
//!     }).collect::<Vec<_>>();
//!
//! assert_eq!(processed_cloud, cloud_copy);
//! ```
//!
//! # Features
//! - r2r_msg — Integration for the ROS2 library [r2r](https://github.com/sequenceplanner/r2r).
//! - rosrust_msg — Integration with the [rosrust](https://github.com/adnanademovic/rosrust) library for ROS1 message types.
//! - (rclrs_msg) — Integration for ROS2 [rclrs](https://github.com/ros2-rust/ros2_rust) but it currently needs [this workaround](https://github.com/stelzo/ros_pointcloud2?tab=readme-ov-file#rclrs-ros2_rust).
//! - derive — Offers implementations for the [`PointConvertible`] trait needed for custom points.
//! - rayon — Parallel iterator support for `_par_iter` functions.
//! - nalgebra — Predefined points offer a nalgebra typed getter for coordinates (e.g. [`xyz`](points::PointXYZ::xyz)).
//! - std *(enabled by default)* — Omit this feature to use this library in no_std environments. ROS integrations and 'rayon' will not work with no_std.
//!
//! # Custom Points
//! Implement [`PointConvertible`] for your point with the `derive` feature or manually.
//!
//! ## Derive (recommended)
//! ```ignore
//! #[derive(Clone, Debug, PartialEq, Copy, Default, PointConvertible)]
//! #[repr(C, align(4))]
//! pub struct MyPointXYZI {
//!     pub x: f32,
//!     pub y: f32,
//!     pub z: f32,
//!     #[rpcl2(rename("i"))]
//!     pub intensity: f32,
//! }
//! ```
//!
//! ## Manual
//! ```
//! use ros_pointcloud2::prelude::*;
//!
//! #[derive(Clone, Debug, PartialEq, Copy, Default)]
//! #[repr(C, align(4))]
//! pub struct MyPointXYZI {
//!     pub x: f32,
//!     pub y: f32,
//!     pub z: f32,
//!     pub intensity: f32,
//! }
//!
//! impl MyPointXYZI {
//!     pub fn new(x: f32, y: f32, z: f32, intensity: f32) -> Self {
//!         Self { x, y, z, intensity }
//!     }
//! }
//!
//! impl From<RPCL2Point<4>> for MyPointXYZI {
//!     fn from(point: RPCL2Point<4>) -> Self {
//!         Self::new(point[0].get(), point[1].get(), point[2].get(), point[3].get())
//!     }
//! }
//!
//! impl From<MyPointXYZI> for RPCL2Point<4> {
//!     fn from(point: MyPointXYZI) -> Self {
//!         [point.x.into(), point.y.into(), point.z.into(), point.intensity.into()].into()
//!     }
//! }
//!
//! unsafe impl PointConvertible<4> for MyPointXYZI {
//!     fn layout() -> LayoutDescription {
//!         LayoutDescription::new(&[
//!             LayoutField::new("x", "f32", 4),
//!             LayoutField::new("y", "f32", 4),
//!             LayoutField::new("z", "f32", 4),
//!             LayoutField::new("intensity", "f32", 4),
//!         ])
//!     }
//! }
//!
//! let first_p = MyPointXYZI::new(1.0, 2.0, 3.0, 0.5);
//! let cloud_points = vec![first_p, MyPointXYZI::new(4.0, 5.0, 6.0, 0.5)];
//! let msg_out = PointCloud2Msg::try_from_iter(cloud_points).unwrap();
//! let cloud_points_out: Vec<MyPointXYZI> = msg_out.try_into_iter().unwrap().collect();
//! assert_eq!(first_p, *cloud_points_out.first().unwrap());
//! ```
#![crate_type = "lib"]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![doc(html_root_url = "https://docs.rs/ros_pointcloud2/0.5.1")]
#![warn(clippy::print_stderr)]
#![warn(clippy::print_stdout)]
#![warn(clippy::unwrap_used)]
#![warn(clippy::expect_used)]
#![warn(clippy::cargo)]
#![warn(clippy::std_instead_of_core)]
#![warn(clippy::alloc_instead_of_core)]
#![warn(clippy::std_instead_of_alloc)]
#![cfg_attr(not(feature = "std"), no_std)]
// Setup an allocator with #[global_allocator]
// see: https://doc.rust-lang.org/std/alloc/trait.GlobalAlloc.html
#![allow(unexpected_cfgs)]

pub mod points;
pub mod prelude;
pub mod ros;

pub mod iterator;

use crate::ros::{HeaderMsg, PointFieldMsg};

use core::str::FromStr;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[macro_use]
extern crate alloc;
use alloc::string::String;
use alloc::vec::Vec;

/// All errors that can occur while converting to or from the message type.
#[derive(Debug)]
pub enum MsgConversionError {
    InvalidFieldFormat,
    #[cfg(feature = "std")]
    UnsupportedFieldType(String),
    #[cfg(not(feature = "std"))]
    UnsupportedFieldType,
    DataLengthMismatch,
    FieldsNotFound(Vec<String>),
    UnsupportedFieldCount,
    NumberConversion,
    ExhaustedSource,
}

impl From<core::num::TryFromIntError> for MsgConversionError {
    fn from(_: core::num::TryFromIntError) -> Self {
        MsgConversionError::NumberConversion
    }
}

impl core::fmt::Display for MsgConversionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            MsgConversionError::InvalidFieldFormat => {
                write!(f, "The field does not match the expected datatype.")
            }
            #[cfg(feature = "std")]
            MsgConversionError::UnsupportedFieldType(datatype) => {
                write!(
                    f,
                    "The field datatype is not supported by the ROS message description: {datatype}"
                )
            }
            #[cfg(not(feature = "std"))]
            MsgConversionError::UnsupportedFieldType => {
                write!(
                    f,
                    "There is an unsupported field type in the ROS message description."
                )
            }
            MsgConversionError::DataLengthMismatch => {
                write!(f, "The length of the byte buffer in the message does not match the expected length computed from the fields, indicating a corrupted or malformed message.")
            }
            MsgConversionError::FieldsNotFound(fields) => {
                write!(f, "Some fields are not found in the message: {fields:?}")
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
            MsgConversionError::ExhaustedSource => {
                write!(
                    f,
                    "The conversion requests more data from the source type than is available."
                )
            }
        }
    }
}

#[allow(clippy::std_instead_of_core)] // will be stable soon (https://github.com/rust-lang/rust/issues/103765)
#[cfg(feature = "std")]
impl std::error::Error for MsgConversionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        None
    }
}

fn system_endian() -> Endian {
    if cfg!(target_endian = "big") {
        Endian::Big
    } else if cfg!(target_endian = "little") {
        Endian::Little
    } else {
        panic!("Unsupported Endian");
    }
}

/// Description of the memory layout of a type with named fields.
#[derive(Clone, Debug)]
pub struct LayoutDescription(Vec<LayoutField>);

impl LayoutDescription {
    pub fn new(fields: &[LayoutField]) -> Self {
        Self(fields.into())
    }
}

/// Enum to describe the field type and size in a padded or unpadded layout.
#[derive(Clone, Debug)]
pub enum LayoutField {
    Field {
        name: alloc::borrow::Cow<'static, str>,
        ty: alloc::borrow::Cow<'static, str>,
        size: usize,
    },
    Padding {
        size: usize,
    },
}

impl LayoutField {
    pub fn new(name: &'static str, ty: &'static str, size: usize) -> Self {
        LayoutField::Field {
            name: name.into(),
            ty: ty.into(),
            size,
        }
    }

    pub fn padding(size: usize) -> Self {
        LayoutField::Padding { size }
    }
}

/// The intermediate point cloud type for ROS integrations.
///
/// To assert consistency, the type should be build with the [`PointCloud2MsgBuilder`].
/// See the offical [ROS message description](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html) for more information on the fields.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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

/// Endianess encoding hint for the message.
#[derive(Default, Clone, Debug, PartialEq, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Endian {
    Big,
    #[default]
    Little,
}

/// Density flag for the message. Writing sparse point clouds is not supported.
#[derive(Default, Clone, Debug, PartialEq, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Denseness {
    #[default]
    Dense,
    Sparse,
}

#[derive(Clone, Debug, PartialEq)]
enum ByteSimilarity {
    Equal,
    Overlapping,
    Different,
}

/// Creating a [`CloudDimensions`] type with the builder pattern to avoid invalid states when using 1-row point clouds.
#[derive(Clone, Debug)]
pub struct CloudDimensionsBuilder(usize);

impl CloudDimensionsBuilder {
    #[must_use]
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
            height: u32::from(self.0 > 0),
        })
    }
}

/// Creating a [`PointCloud2Msg`] with the builder pattern to avoid invalid states.
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
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    #[must_use]
    pub fn header(mut self, header: HeaderMsg) -> Self {
        self.header = header;
        self
    }

    #[must_use]
    pub fn width(mut self, width: u32) -> Self {
        self.width = width;
        self
    }

    #[must_use]
    pub fn fields(mut self, fields: Vec<PointFieldMsg>) -> Self {
        self.fields = fields;
        self
    }

    #[must_use]
    pub fn endian(mut self, is_big_endian: bool) -> Self {
        self.is_big_endian = is_big_endian;
        self
    }

    #[must_use]
    pub fn point_step(mut self, point_step: u32) -> Self {
        self.point_step = point_step;
        self
    }

    #[must_use]
    pub fn row_step(mut self, row_step: u32) -> Self {
        self.row_step = row_step;
        self
    }

    #[must_use]
    pub fn data(mut self, data: Vec<u8>) -> Self {
        self.data = data;
        self
    }

    #[must_use]
    pub fn dense(mut self, is_dense: bool) -> Self {
        self.is_dense = is_dense;
        self
    }

    /// Build the [`PointCloud2Msg`] from the builder.
    ///
    /// # Errors
    /// Returns an error if the fields are empty, the field count is not 1, the field format is invalid, the data length does not match the point step, or the field size is too large.
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

/// Dimensions of the point cloud as width and height.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CloudDimensions {
    pub width: u32,
    pub height: u32,
}

fn ordered_field_names<const N: usize, C: PointConvertible<N>>() -> Vec<String> {
    C::layout()
        .0
        .iter()
        .filter(|field| {
            matches!(
                field,
                LayoutField::Field {
                    name: _,
                    ty: _,
                    size: _,
                }
            )
        })
        .map(|field| match field {
            LayoutField::Field {
                name,
                ty: _,
                size: _,
            } => name.as_ref().into(),
            _ => unreachable!("Fields must be filtered before."),
        })
        .collect()
}

impl PointCloud2Msg {
    #[inline]
    fn byte_similarity<const N: usize, C>(&self) -> Result<ByteSimilarity, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        let field_names = ordered_field_names::<N, C>();
        let target_layout = KnownLayoutInfo::try_from(C::layout())?;

        debug_assert!(field_names.len() <= target_layout.fields.len());
        debug_assert!(self.fields.len() >= target_layout.fields.len());

        let mut offset: u32 = 0;
        let mut field_counter = 0;
        for f in target_layout.fields.iter() {
            match f {
                PointField::Field {
                    datatype,
                    size,
                    count,
                } => {
                    if field_counter >= self.fields.len() || field_counter >= field_names.len() {
                        return Err(MsgConversionError::ExhaustedSource);
                    }

                    let msg_f = unsafe { self.fields.get_unchecked(field_counter) };
                    let f_translated = unsafe { field_names.get_unchecked(field_counter) };
                    field_counter += 1;

                    if msg_f.name != *f_translated
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

    /// Create a [`PointCloud2Msg`] from any iterable type that implements [`PointConvertible`].
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
        let (mut cloud, point_step) = {
            let point: RPCL2Point<N> = C::default().into();
            debug_assert!(point.fields.len() == N);

            let field_names = crate::ordered_field_names::<N, C>();
            debug_assert!(field_names.len() == N);

            let mut pdata_offsets_acc: u32 = 0;
            let mut fields = vec![PointFieldMsg::default(); N];
            let field_count: u32 = 1;
            for ((pdata_entry, field_name), field_val) in point
                .fields
                .into_iter()
                .zip(field_names.into_iter())
                .zip(fields.iter_mut())
            {
                let datatype_code = pdata_entry.datatype.into();
                let _ = FieldDatatype::try_from(datatype_code)?;

                *field_val = PointFieldMsg {
                    name: field_name,
                    offset: pdata_offsets_acc,
                    datatype: datatype_code,
                    count: 1,
                };

                pdata_offsets_acc += field_count * pdata_entry.datatype.size() as u32;
            }

            (
                PointCloud2MsgBuilder::new()
                    .fields(fields)
                    .point_step(pdata_offsets_acc),
                pdata_offsets_acc,
            )
        };
        let mut cloud_width = 0;

        iterable.into_iter().for_each(|pointdata| {
            let point: RPCL2Point<N> = pointdata.into();

            point.fields.iter().for_each(|pdata| {
                let truncated_bytes = unsafe {
                    core::slice::from_raw_parts(pdata.bytes.as_ptr(), pdata.datatype.size())
                };
                cloud.data.extend_from_slice(truncated_bytes);
            });

            cloud_width += 1;
        });

        cloud = cloud.width(cloud_width);
        cloud = cloud.row_step(cloud_width * point_step);

        cloud.build()
    }

    /// Create a PointCloud2Msg from a parallel iterator. Requires the `rayon` and `derive` feature to be enabled.
    #[cfg(feature = "rayon")]
    #[cfg_attr(docsrs, doc(cfg(feature = "rayon")))]
    pub fn try_from_par_iter<const N: usize, C>(
        iterable: impl rayon::iter::ParallelIterator<Item = C>,
    ) -> Result<Self, MsgConversionError>
    where
        C: PointConvertible<N> + Send + Sync,
    {
        Self::try_from_vec(iterable.collect::<Vec<_>>())
    }

    /// Create a [`PointCloud2Msg`] from a Vec of points.
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
    ///
    /// # Errors
    /// Returns an error if the byte buffer does not match the expected layout or the message contains other discrepancies.
    pub fn try_from_vec<const N: usize, C>(vec: Vec<C>) -> Result<Self, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        match (system_endian(), Endian::default()) {
            (Endian::Big, Endian::Big) | (Endian::Little, Endian::Little) => {
                let (mut cloud, point_step) = {
                    let point: RPCL2Point<N> = C::default().into();
                    debug_assert!(point.fields.len() == N);

                    let field_names = crate::ordered_field_names::<N, C>();
                    debug_assert!(field_names.len() == N);

                    let layout = KnownLayoutInfo::try_from(C::layout())?;
                    debug_assert!(field_names.len() <= layout.fields.len());

                    let mut offset = 0;
                    let mut fields: Vec<PointFieldMsg> = Vec::with_capacity(field_names.len());
                    for f in layout.fields.into_iter() {
                        match f {
                            PointField::Field {
                                datatype,
                                size,
                                count,
                            } => {
                                fields.push(PointFieldMsg {
                                    name: field_names[fields.len()].clone(),
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

                    (
                        PointCloud2MsgBuilder::new()
                            .fields(fields)
                            .point_step(offset),
                        offset,
                    )
                };

                let bytes_total = vec.len() * point_step as usize;
                cloud.data.resize(bytes_total, u8::default());
                let raw_data: *mut C = cloud.data.as_mut_ptr() as *mut C;
                unsafe {
                    core::ptr::copy_nonoverlapping(
                        vec.as_ptr().cast::<u8>(),
                        raw_data.cast::<u8>(),
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

    /// Convert the [`PointCloud2Msg`] to a Vec of points.
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
    ///
    /// # Errors
    /// Returns an error if the byte buffer does not match the expected layout or the message contains other discrepancies.
    pub fn try_into_vec<const N: usize, C>(self) -> Result<Vec<C>, MsgConversionError>
    where
        C: PointConvertible<N>,
    {
        match (system_endian(), self.endian) {
            (Endian::Big, Endian::Big) | (Endian::Little, Endian::Little) => {
                let bytematch = match self.byte_similarity::<N, C>()? {
                    ByteSimilarity::Equal => true,
                    ByteSimilarity::Overlapping => false,
                    ByteSimilarity::Different => return Ok(self.try_into_iter()?.collect()),
                };

                let cloud_width = self.dimensions.width as usize;
                let point_step = self.point_step as usize;
                let mut vec: Vec<C> = Vec::with_capacity(cloud_width);
                if bytematch {
                    unsafe {
                        core::ptr::copy_nonoverlapping(
                            self.data.as_ptr(),
                            vec.as_mut_ptr().cast::<u8>(),
                            self.data.len(),
                        );
                        vec.set_len(cloud_width);
                    }
                } else {
                    unsafe {
                        for i in 0..cloud_width {
                            let point_ptr = self.data.as_ptr().add(i * point_step).cast::<C>();
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

    /// Convert the [`PointCloud2Msg`] to an iterator.
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
    /// # Errors
    /// Returns an error if the byte buffer does not match the expected layout or the message contains other discrepancies.
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
    #[cfg_attr(docsrs, doc(cfg(feature = "rayon")))]
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

/// Internal point representation. It is used to store the point data entries.
///
/// In each iteration, an internal point representation is converted to the desired point type.
/// Implement the `From` traits for your point type to use the conversion.
///
/// See the [`PointConvertible`] trait for more information.
pub struct RPCL2Point<const N: usize> {
    fields: [PointData; N],
}

impl<const N: usize> core::ops::Index<usize> for RPCL2Point<N> {
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
/// Implement this trait for your custom point you want to read or write in the message.
/// It is strongly recommended to enable the `derive` feature and use the `#[derive(PointConvertible)]` macro.
/// This prevents common errors when implementing this trait by hand.
///
/// Be aware that Rust does not guarantee the memory layout of structs. Learn more [here](https://doc.rust-lang.org/reference/type-layout.html).
/// To make layouting more predictable and thus faster for C++ node interactions, use the `#[repr(C)]` attribute on your struct.
/// An example for diverging point layouts with padding can be seen in the source code of [this](points::PointXYZRGBA::layout) implementation.
///
/// The generic parameter `N` is the number of fields in the point type. There can be more (hidden) fields that pad the layout but they do not count for the N.
/// For
///
/// # Derive
/// ```ignore
/// use ros_pointcloud2::prelude::*;
///
/// #[derive(Clone, Debug, PartialEq, Copy, Default, PointConvertible)]
/// #[repr(C, align(4))]
/// pub struct MyPointXYZL {
///     pub x: f32,
///     pub y: f32,
///     pub z: f32,
///     #[rpcl2(rename("l"))]
///     pub label: u8,
/// }
/// ```
///
/// # Manual
/// ```
/// use ros_pointcloud2::prelude::*;
///
/// #[derive(Clone, Debug, PartialEq, Copy, Default)]
/// #[repr(C, align(4))]
/// pub struct MyPointXYZL {
///     pub x: f32,
///     pub y: f32,
///     pub z: f32,
///     pub label: u8,
/// }
///
/// impl From<MyPointXYZL> for RPCL2Point<4> {
///     fn from(point: MyPointXYZL) -> Self {
///         [point.x.into(), point.y.into(), point.z.into(), point.label.into()].into()
///     }
/// }
///
/// impl From<RPCL2Point<4>> for MyPointXYZL {
///     fn from(point: RPCL2Point<4>) -> Self {
///         Self {
///             x: point[0].get(),
///             y: point[1].get(),
///             z: point[2].get(),
///             label: point[3].get(),
///         }
///     }
/// }
///
/// unsafe impl PointConvertible<4> for MyPointXYZL {
///     fn layout() -> LayoutDescription {
///         LayoutDescription::new(&[
///             LayoutField::new("x", "f32", 4),
///             LayoutField::new("y", "f32", 4),
///             LayoutField::new("z", "f32", 4),
///             LayoutField::new("l", "u8", 1),
///             LayoutField::padding(3),
///         ])
///     }
/// }
/// ```
/// # Safety
/// The layout is used for raw memory interpretation, where safety can not be guaranteed by the compiler.
/// Take care when implementing the layout, especially in combination with `#[repr]` or use the `derive` feature when possible to prevent common errors.
pub unsafe trait PointConvertible<const N: usize>:
    From<RPCL2Point<N>> + Into<RPCL2Point<N>> + Default + Sized
{
    fn layout() -> LayoutDescription;
}

#[derive(Debug, Clone)]
enum PointField {
    Padding(u32),
    Field { size: u32, datatype: u8, count: u32 },
}

#[derive(Debug, Clone)]
struct KnownLayoutInfo {
    fields: Vec<PointField>,
}

impl TryFrom<LayoutField> for PointField {
    type Error = MsgConversionError;

    fn try_from(f: LayoutField) -> Result<Self, Self::Error> {
        match f {
            LayoutField::Field { name: _, ty, size } => {
                let typename: String = ty.into_owned().to_lowercase();
                let datatype = FieldDatatype::from_str(typename.as_str())?;
                Ok(Self::Field {
                    size: size.try_into()?,
                    datatype: datatype.into(),
                    count: 1,
                })
            }
            LayoutField::Padding { size } => Ok(Self::Padding(size.try_into()?)),
        }
    }
}

impl TryFrom<LayoutDescription> for KnownLayoutInfo {
    type Error = MsgConversionError;

    fn try_from(t: LayoutDescription) -> Result<Self, Self::Error> {
        let fields: Vec<PointField> =
            t.0.into_iter()
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
/// let pdata = PointData::new(original_data);
/// let my_data: f64 = pdata.get();
/// ```
#[derive(Debug, Clone, Copy)]
pub struct PointData {
    bytes: [u8; core::mem::size_of::<f64>()],
    endian: Endian,
    datatype: FieldDatatype,
}

impl Default for PointData {
    fn default() -> Self {
        Self {
            bytes: [u8::default(); core::mem::size_of::<f64>()],
            datatype: FieldDatatype::F32,
            endian: Endian::default(),
        }
    }
}

impl PointData {
    /// Create a new [`PointData`] from a value.
    ///
    /// # Example
    /// ```
    /// let pdata = ros_pointcloud2::PointData::new(1.0);
    /// ```
    #[inline]
    pub fn new<T: FromBytes>(value: T) -> Self {
        Self {
            bytes: value.into().raw(),
            datatype: T::field_datatype(),
            ..Default::default()
        }
    }

    #[inline]
    fn from_buffer(data: &[u8], offset: usize, datatype: FieldDatatype, endian: Endian) -> Self {
        debug_assert!(data.len() >= offset + datatype.size());
        let mut bytes = [u8::default(); core::mem::size_of::<f64>()];
        unsafe {
            let data_ptr = data.as_ptr().add(offset);
            core::ptr::copy_nonoverlapping(data_ptr, bytes.as_mut_ptr(), datatype.size());
        }

        Self {
            bytes,
            endian,
            datatype,
        }
    }

    /// Get the numeric value from the [`PointData`] description.
    ///
    /// # Example
    /// ```
    /// let original_data: f64 = 1.0;
    /// let pdata = ros_pointcloud2::PointData::new(original_data);
    /// let my_data: f64 = pdata.get();
    /// ```
    #[must_use]
    pub fn get<T: FromBytes>(&self) -> T {
        match self.endian {
            Endian::Big => T::from_be_bytes(PointDataBuffer::new(self.bytes)),
            Endian::Little => T::from_le_bytes(PointDataBuffer::new(self.bytes)),
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

/// Datatypes from the [`PointFieldMsg`].
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
    /// To make it easy to work with and avoid packing code, the [`RGB`](points::RGB) union is supported here and handled like a f32.
    RGB,
}

impl FieldDatatype {
    #[must_use]
    pub fn size(&self) -> usize {
        match self {
            FieldDatatype::U8 => core::mem::size_of::<u8>(),
            FieldDatatype::U16 => core::mem::size_of::<u16>(),
            FieldDatatype::U32 => core::mem::size_of::<u32>(),
            FieldDatatype::I8 => core::mem::size_of::<i8>(),
            FieldDatatype::I16 => core::mem::size_of::<i16>(),
            FieldDatatype::I32 => core::mem::size_of::<i32>(),
            FieldDatatype::F32 | FieldDatatype::RGB => core::mem::size_of::<f32>(), // packed in f32
            FieldDatatype::F64 => core::mem::size_of::<f64>(),
        }
    }
}

impl core::str::FromStr for FieldDatatype {
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
            #[cfg(feature = "std")]
            _ => Err(MsgConversionError::UnsupportedFieldType(s.into())),
            #[cfg(not(feature = "std"))]
            _ => Err(MsgConversionError::UnsupportedFieldType),
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
            #[cfg(feature = "std")]
            _ => Err(MsgConversionError::UnsupportedFieldType(value.to_string())),
            #[cfg(not(feature = "std"))]
            _ => Err(MsgConversionError::UnsupportedFieldType),
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
            FieldDatatype::F32 | FieldDatatype::RGB => 7, // RGB is marked as f32 in the buffer
            FieldDatatype::F64 => 8,
        }
    }
}

impl TryFrom<&ros::PointFieldMsg> for FieldDatatype {
    type Error = MsgConversionError;

    fn try_from(value: &ros::PointFieldMsg) -> Result<Self, Self::Error> {
        Self::try_from(value.datatype)
    }
}

/// Byte buffer alias for endian-aware point data reading and writing.
///
/// It uses a fixed size buffer of 8 bytes since the largest supported datatype for [`PointFieldMsg`] is f64.
pub struct PointDataBuffer([u8; 8]);

impl core::ops::Index<usize> for PointDataBuffer {
    type Output = u8;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl PointDataBuffer {
    #[must_use]
    pub fn new(data: [u8; 8]) -> Self {
        Self(data)
    }

    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }

    #[must_use]
    pub fn raw(self) -> [u8; 8] {
        self.0
    }

    #[must_use]
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
/// All [`PointFieldMsg`] types are supported.
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

mod test {
    #![allow(clippy::unwrap_used)]

    use crate::prelude::*;

    #[derive(Debug, Default, Clone, PartialEq)]
    #[repr(C)]
    struct PointA {
        x: f32,
        y: f32,
        z: f32,
        intensity: f32,
        t: u32,
        reflectivity: u16,
        ring: u16,
        ambient: u16,
        range: u32,
    }

    impl From<RPCL2Point<9>> for PointA {
        fn from(point: RPCL2Point<9>) -> Self {
            Self::new(point[0].get(), point[1].get(), point[2].get())
        }
    }

    impl From<PointA> for RPCL2Point<9> {
        fn from(point: PointA) -> Self {
            [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                point.intensity.into(),
                point.t.into(),
                point.reflectivity.into(),
                point.ring.into(),
                point.ambient.into(),
                point.range.into(),
            ]
            .into()
        }
    }

    unsafe impl PointConvertible<9> for PointA {
        fn layout() -> LayoutDescription {
            LayoutDescription::new(&[
                LayoutField::new("x", "f32", 4),
                LayoutField::new("y", "f32", 4),
                LayoutField::new("z", "f32", 4),
                LayoutField::new("intensity", "f32", 4),
                LayoutField::new("t", "u32", 4),
                LayoutField::new("reflectivity", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("ring", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("ambient", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("range", "u32", 4),
            ])
        }
    }

    impl PointA {
        fn new(x: f32, y: f32, z: f32) -> Self {
            Self {
                x,
                y,
                z,
                intensity: 0.0,
                t: 0,
                reflectivity: 0,
                ring: 0,
                ambient: 0,
                range: 0,
            }
        }
    }

    #[derive(Debug, Clone, Default, PartialEq)]
    #[repr(C)]
    struct PointB {
        pub x: f32,
        pub y: f32,
        pub z: f32,
        pub t: u32,
    }

    impl PointB {
        fn new(x: f32, y: f32, z: f32) -> Self {
            Self { x, y, z, t: 0 }
        }
    }

    impl From<RPCL2Point<4>> for PointB {
        fn from(point: RPCL2Point<4>) -> Self {
            Self::new(point[0].get(), point[1].get(), point[2].get())
        }
    }

    impl From<PointB> for RPCL2Point<4> {
        fn from(point: PointB) -> Self {
            [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                point.t.into(),
            ]
            .into()
        }
    }

    unsafe impl PointConvertible<4> for PointB {
        fn layout() -> LayoutDescription {
            LayoutDescription::new(&[
                LayoutField::new("x", "f32", 4),
                LayoutField::new("y", "f32", 4),
                LayoutField::new("z", "f32", 4),
                LayoutField::new("t", "u32", 4),
            ])
        }
    }

    #[derive(Debug, Clone, Default, PartialEq)]
    #[repr(C)]
    struct PointD {
        x: f32,
        y: f32,
        z: f32,
        t: u32,
        ring: u16,
        range: u32,
        signal: u16,
        reflectivity: u16,
        near_ir: u16,
    }

    impl From<RPCL2Point<9>> for PointD {
        fn from(point: RPCL2Point<9>) -> Self {
            Self::new(point[0].get(), point[1].get(), point[2].get())
        }
    }

    impl From<PointD> for RPCL2Point<9> {
        fn from(point: PointD) -> Self {
            [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                point.t.into(),
                point.ring.into(),
                point.range.into(),
                point.signal.into(),
                point.reflectivity.into(),
                point.near_ir.into(),
            ]
            .into()
        }
    }

    unsafe impl PointConvertible<9> for PointD {
        fn layout() -> LayoutDescription {
            LayoutDescription::new(&[
                LayoutField::new("x", "f32", 4),
                LayoutField::new("y", "f32", 4),
                LayoutField::new("z", "f32", 4),
                LayoutField::new("t", "u32", 4),
                LayoutField::new("ring", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("range", "u32", 4),
                LayoutField::new("signal", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("reflectivity", "u16", 2),
                LayoutField::padding(2),
                LayoutField::new("near_ir", "u16", 2),
                LayoutField::padding(2),
            ])
        }
    }

    impl PointD {
        fn new(x: f32, y: f32, z: f32) -> Self {
            Self {
                x,
                y,
                z,
                t: 0,
                ring: 0,
                range: 0,
                signal: 0,
                reflectivity: 0,
                near_ir: 0,
            }
        }
    }

    #[test]
    fn subtype_iterator_fallback() {
        let cloud_a = PointCloud2Msg::try_from_iter(vec![
            PointA::new(1.0, 2.0, 3.0),
            PointA::new(4.0, 5.0, 6.0),
            PointA::new(7.0, 8.0, 9.0),
        ])
        .unwrap();

        let cloud_c: PointB = cloud_a.clone().try_into_iter().unwrap().next().unwrap();
        assert_eq!(cloud_c, PointB::new(1.0, 2.0, 3.0));

        let cloud_b: Vec<PointB> = cloud_a.try_into_vec().unwrap();
        assert_eq!(cloud_b[0], PointB::new(1.0, 2.0, 3.0));
        assert_eq!(cloud_b[1], PointB::new(4.0, 5.0, 6.0));
        assert_eq!(cloud_b[2], PointB::new(7.0, 8.0, 9.0));
    }
}
