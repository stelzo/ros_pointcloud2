//! This library provides the [`PointCloud2Msg`] type, which implements conversions to and from slices and iterators.
//! All conversions are optimized for performance and minimal memory operations. They range from zero-copy to fully owned allocations.
//!
//! Vector conversions are optimal for moving data around and a good default choice.
//! - [`try_into_slice`](PointCloud2Msg::try_into_slice) creates a slice view
//! - [`try_from_slice`](PointCloud2Msg::try_from_slice) creates a message from a slice
//!
//! - [`try_into_vec`](PointCloud2Msg::try_into_vec) creates an owned Vec
//! - [`try_from_vec`](PointCloud2Msg::try_from_vec) creates a message from an owned Vec
//!
//! When the layout is compatible, the following functions reuse the underlying buffer without copies but error on unsupported layouts.
//! They are internally used by the non-strict versions when possible, but if you need to ensure zero-copy (and you can guarantee the memory layout) you can use them directly.
//! - [`try_into_slice_strict`](PointCloud2Msg::try_into_slice_strict) zero-copy slice view, errors on missmatched layouts
//! - [`try_from_vec_strict`](PointCloud2Msg::try_from_vec_strict) zero-copy from owned Vec, errors on missmatched layouts
//!
//! Iterators are useful for more sophisticated point types, layout missmatches or when doing a lot of processing per point.
//! They offer maximum flexibility and fully support all edge cases of the PointCloud2 message.
//! While, in theory, this comes at the cost of performance, they often get compiled to similar performant binaries as the slice and Vec conversions when used with simple point types thanks to SIMD and other optimizations.
//! - [`try_from_iter`](PointCloud2Msg::try_from_iter) allocates a new message from an iterator
//! - [`try_into_iter`](PointCloud2Msg::try_into_iter) iterator over points in the message
//!
//! They feature predictable performance but they do not scale well with large clouds. Learn more about that in the [performance section](https://github.com/stelzo/ros_pointcloud2?tab=readme-ov-file#performance) of the repository.
//! The iterators are useful when your conversions are more complex than a simple copy or the cloud is small enough.
//!
//! When the cloud is getting larger or you are doing a lot of processing per point, turn on the `rayon` feature and switch to the parallel iterators.
//! - [`try_into_par_iter`](PointCloud2Msg::try_into_par_iter) requires `rayon` feature
//! - [`try_from_par_iter`](PointCloud2Msg::try_from_par_iter) requires `rayon` feature
//!
//! They often outperform all other methods, even for smaller clouds thanks to the rayon optimizations but come at the cost of higher memory and CPU usage.
//!
//! # Support for ROS client crates
//!
//! Support for client crates is provided via consumer-side macros that generate conversions between `PointCloud2Msg` and the client crate's message types.
//!
//! Simply add the client crate to your `Cargo.toml` and invoke the corresponding macro in your crate scope.
//!
//! ## Quick examples
//!
//! Add the client crate to your `Cargo.toml`:
//!
//! ```toml
//! [dependencies]
//! ros_pointcloud2 = "*"
//!
//! r2r = "0.9"
//! # ... or maybe for ROS1:
//! rosrust = "0.9"
//!
//! nalgebra = "0.34"
//! ```
//!
//! Then invoke the macro in your crate root or tests to generate conversions:
//! - r2r: [`impl_pointcloud2_for_r2r!`]
//! - rclrs: [`impl_pointcloud2_for_rclrs!`]
//! - rosrust: [`impl_pointcloud2_for_rosrust!`]
//! - ros2-client: [`impl_pointcloud2_for_ros2_interfaces_jazzy_serde!`]
//! - roslibrust ROS1: [`impl_pointcloud2_for_roslibrust_ros1!`]
//! - roslibrust ROS2: [`impl_pointcloud2_for_roslibrust_ros2!`]
//!
//! The roslibrust macros need to be invoked with the crate root where the messages are included via `include!`.
//!
//! For example: `impl_pointcloud2_for_roslibrust_ros2!(crate);`
//!
//! Also, indicate the following dependencies to your linker inside the `package.xml` of your package if your crate of choice uses the xml to link messages.
//!
//! ```xml
//! <depend>std_msgs</depend>
//! <depend>sensor_msgs</depend>
//! <depend>builtin_interfaces</depend>
//! ```
//!
//! There is also [nalgebra](https://docs.rs/nalgebra/latest/nalgebra/) support to convert common point types to nalgebra `Point3` type [`impl_pointxyz_for_nalgebra!`].
//!
//! ```ignore
//! ros_pointcloud2::impl_pointxyz_for_nalgebra!();
//!
//! use ros_pointcloud2::points::PointXYZI;
//! use ros_pointcloud2::impl_nalgebra::AsNalgebra;
//! use nalgebra::Point3;
//! let p_xyzi = PointXYZI::new(4.0, 5.0, 6.0, 7.0);
//! assert_eq!(AsNalgebra::xyz(&p_xyzi), nalgebra::Point3::new(4.0, 5.0, 6.0));
//! ```
//!
//! Common point types like [`PointXYZ`](points::PointXYZ) or [`PointXYZI`](points::PointXYZI) are provided. See the full list [`here`](points).
//!
//! You can easily add any additional custom type. See [`custom_enum_field_filter`] for a detailed example.
//!
//! # Minimal Example
//! ```
//! use ros_pointcloud2::prelude::*;
//!
//! let cloud_points = vec![
//!     PointXYZI::new(9.6, 42.0, -6.2, 0.1),
//!     PointXYZI::new(46.0, 5.47, 0.5, 0.1),
//! ];
//!
//! let out_msg = PointCloud2Msg::try_from_slice(&cloud_points).unwrap();
//!
//! // Add your ROS crate impl macro in your crate to enable conversions.
//! // For example, for r2r:
//! // impl_pointcloud2_for_r2r!();
//!
//! // Convert to your ROS crate message type.
//! // let msg = impl_r2r::from_pointcloud2_msg(out_msg);
//! // Publish ...
//!
//! // ... now incoming from a topic.
//! // let in_msg = impl_r2r::to_pointcloud2_msg(msg);
//! let in_msg = out_msg;
//!
//! let processed_cloud = in_msg.try_into_iter().unwrap()
//!     .map(|point: PointXYZ| { // Define the type you want to map the data to.
//!         // Access the data like a normal struct.
//!         PointXYZI::new(point.x, point.y, point.z, 0.1)
//!     }).collect::<Vec<_>>();
//!
//! assert_eq!(processed_cloud, cloud_points);
//! ```
//!
//! # Features
//! - std *(enabled by default)* — Omit this feature to use this library in no_std environments. ROS integrations and `rayon` will not work with no_std.
//! - strict-type-check *(enabled by default)* — When disabled, allows byte conversions between size-compatible types like i32 and f32. Packed RGB fields are specially handled and allowed.
//! - derive — Offers implementations for the [`PointConvertible`] trait needed for custom points.
//! - serde — Enables serde serialization and deserialization for [`PointCloud2Msg`] and related types.
//! - rkyv — Enables rkyv serialization and deserialization for [`PointCloud2Msg`] and related types.
//! - rayon — Parallel iterator support for `*_par_iter` functions.
//!
//! # Custom Points
//! Implement [`PointConvertible`] for your point with the `derive` feature or manually.
//!
//! ## Derive
//! The derive macro supports renaming fields to match the message field names.
//!
//! ```ignore
//! #[derive(Clone, Debug, PartialEq, Copy, Default, PointConvertible)]
//! #[repr(C, align(4))]
//! pub struct MyPointXYZI {
//!     pub x: f32,
//!     pub y: f32,
//!     pub z: f32,
//!     #[ros(remap("i"))]
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
//! impl From<IPoint<4>> for MyPointXYZI {
//!     fn from(point: IPoint<4>) -> Self {
//!         Self::new(point[0].get(), point[1].get(), point[2].get(), point[3].get())
//!     }
//! }
//!
//! impl From<MyPointXYZI> for IPoint<4> {
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
//! let msg_out = PointCloud2Msg::try_from_iter(&cloud_points).unwrap();
//! let cloud_points_out: Vec<MyPointXYZI> = msg_out.try_into_iter().unwrap().collect();
//! assert_eq!(first_p, *cloud_points_out.first().unwrap());
//! ```
#![crate_type = "lib"]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![doc(html_root_url = "https://docs.rs/ros_pointcloud2/1.0.0-rc1")]
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

#[cfg(doc)]
#[doc = concat!("Custom Field Type Example (docs only).\n\n```rust\n", include_str!("../examples/custom_enum_field_filter.rs"), "\n```")]
pub mod custom_enum_field_filter {}

pub mod points;
pub mod prelude;
pub mod ros;

pub mod iterator;

#[cfg(test)]
mod tests;

use crate::ros::{HeaderMsg, PointFieldMsg};
use core::str::FromStr;

#[macro_use]
extern crate alloc;
use alloc::string::String;
use alloc::vec::Vec;

/// All errors that can occur while converting to or from the message type.
#[derive(Debug)]
pub enum ConversionError {
    InvalidFieldFormat,
    UnsupportedFieldType(String),
    DataLengthMismatch,
    FieldsNotFound(Vec<String>),
    UnsupportedFieldCount,
    NumberConversion,
    TypeMismatch {
        stored: FieldDatatype,
        requested: FieldDatatype,
    },
    ExhaustedSource,
    UnalignedBuffer,
    VecElementSizeMismatch {
        element_size: usize,
        expected_point_step: usize,
    },
    UnsupportedSliceView,
}

impl From<core::num::TryFromIntError> for ConversionError {
    fn from(_: core::num::TryFromIntError) -> Self {
        ConversionError::NumberConversion
    }
}

impl core::fmt::Display for ConversionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConversionError::InvalidFieldFormat => {
                write!(f, "The field does not match the expected datatype.")
            }
            ConversionError::UnsupportedFieldType(datatype) => {
                write!(
                    f,
                    "The field datatype is not supported by the ROS message description: {datatype}"
                )
            }
            ConversionError::DataLengthMismatch => {
                write!(f, "The length of the byte buffer in the message does not match the expected length computed from the fields, indicating a corrupted or malformed message.")
            }
            ConversionError::FieldsNotFound(fields) => {
                write!(f, "Some fields are not found in the message: {fields:?}")
            }
            ConversionError::UnsupportedFieldCount => {
                write!(
                    f,
                    "Only field_count 1 is supported for reading and writing."
                )
            }
            ConversionError::NumberConversion => {
                write!(f, "The number is too large to be converted into a PointCloud2 supported datatype.")
            }
            ConversionError::TypeMismatch { stored, requested } => {
                write!(
                    f,
                    "Stored datatype {:?} is not compatible with requested datatype {:?}.",
                    stored, requested
                )
            }
            ConversionError::ExhaustedSource => {
                write!(
                    f,
                    "The conversion requests more data from the source type than is available."
                )
            }
            ConversionError::UnalignedBuffer => {
                write!(f, "The underlying byte buffer is not properly aligned for the requested slice type.")
            }
            ConversionError::VecElementSizeMismatch {
                element_size,
                expected_point_step,
            } => {
                write!(f, "The input Vec element size ({element_size}) does not match the expected point_step ({expected_point_step}); ownership cannot be transferred without copying.")
            }
            ConversionError::UnsupportedSliceView => {
                write!(f, "The message layout cannot be viewed as a contiguous slice of the requested point type (stride or layout mismatch).")
            }
        }
    }
}

impl core::error::Error for ConversionError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
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
        name: &'static str,
        ty: &'static str,
        size: usize,
    },
    Padding {
        size: usize,
    },
}

impl LayoutField {
    pub fn new(name: &'static str, ty: &'static str, size: usize) -> Self {
        LayoutField::Field { name, ty, size }
    }

    pub fn padding(size: usize) -> Self {
        LayoutField::Padding { size }
    }
}

/// The intermediate point cloud type.
///
/// To assert consistency, the type should be built with the [`PointCloud2MsgBuilder`].
/// The builder performs basic validation (e.g. that `fields` is non-empty, each `PointFieldMsg.count == 1`,
/// `point_step` is large enough for the configured fields, and `data.len()` matches the `point_step`).
///
/// Example
/// ```rust
/// use ros_pointcloud2::prelude::*;
///
/// let header = HeaderMsg { seq: 1, stamp: TimeMsg { sec: 0, nanosec: 0 }, frame_id: "frame".into() };
/// let fields = vec![
///     PointFieldMsg { name: "x".into(), offset: 0, datatype: FieldDatatype::F32.into(), count: 1 },
///     PointFieldMsg { name: "y".into(), offset: 4, datatype: FieldDatatype::F32.into(), count: 1 },
///     PointFieldMsg { name: "z".into(), offset: 8, datatype: FieldDatatype::F32.into(), count: 1 },
/// ];
///
/// let data = vec![0u8; 12]; // one point containing 3 f32 values
///
/// let msg = PointCloud2MsgBuilder::new()
///     .with_header(header)
///     .with_fields(fields)
///     .with_endian(Endian::Little)
///     .with_point_step(12)
///     .with_row_step(12)
///     .with_data(data)
///     .with_dense(Denseness::Dense)
///     .with_width(1)
///     .build()
///     .unwrap();
///
/// assert_eq!(msg.point_step, 12);
/// assert_eq!(msg.dimensions.len(), 1);
/// ```
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub enum Endian {
    Big,
    #[default]
    Little,
}

/// Density flag for the message. Writing sparse point clouds is not supported.
#[derive(Default, Clone, Debug, PartialEq, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "rkyv", derive(rkyv::Archive, rkyv::Serialize))]
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

    pub fn build(self) -> Result<CloudDimensions, ConversionError> {
        let width = match u32::try_from(self.0) {
            Ok(w) => w,
            Err(_) => return Err(ConversionError::NumberConversion),
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
    endian: Endian,
    point_step: u32,
    row_step: u32,
    data: Vec<u8>,
    dense: Denseness,
}

impl PointCloud2MsgBuilder {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    #[must_use]
    pub fn with_header(mut self, header: HeaderMsg) -> Self {
        self.header = header;
        self
    }

    #[deprecated(since = "1.0.0", note = "use `with_header` instead")]
    #[doc(hidden)]
    pub fn header(self, header: HeaderMsg) -> Self {
        self.with_header(header)
    }

    #[must_use]
    pub fn with_width(mut self, width: u32) -> Self {
        self.width = width;
        self
    }

    #[deprecated(since = "1.0.0", note = "use `with_width` instead")]
    #[doc(hidden)]
    pub fn width(self, width: u32) -> Self {
        self.with_width(width)
    }

    #[must_use]
    pub fn with_fields(mut self, fields: Vec<PointFieldMsg>) -> Self {
        self.fields = fields;
        self
    }

    #[deprecated(since = "1.0.0", note = "use `with_fields` instead")]
    #[doc(hidden)]
    pub fn fields(self, fields: Vec<PointFieldMsg>) -> Self {
        self.with_fields(fields)
    }

    #[must_use]
    pub fn with_endian(mut self, endian: Endian) -> Self {
        self.endian = endian;
        self
    }

    #[deprecated(since = "1.0.0", note = "use `with_endian` instead")]
    #[doc(hidden)]
    pub fn endian(self, is_big_endian: bool) -> Self {
        self.with_endian(if is_big_endian {
            Endian::Big
        } else {
            Endian::Little
        })
    }

    #[must_use]
    pub fn with_point_step(mut self, point_step: u32) -> Self {
        self.point_step = point_step;
        self
    }

    #[deprecated(since = "1.0.0", note = "use `with_point_step` instead")]
    #[doc(hidden)]
    pub fn point_step(self, point_step: u32) -> Self {
        self.with_point_step(point_step)
    }

    #[must_use]
    pub fn with_row_step(mut self, row_step: u32) -> Self {
        self.row_step = row_step;
        self
    }

    #[deprecated(since = "1.0.0", note = "use `with_row_step` instead")]
    #[doc(hidden)]
    pub fn row_step(self, row_step: u32) -> Self {
        self.with_row_step(row_step)
    }

    #[must_use]
    pub fn with_data(mut self, data: Vec<u8>) -> Self {
        self.data = data;
        self
    }

    #[deprecated(since = "1.0.0", note = "use `with_data` instead")]
    #[doc(hidden)]
    pub fn data(self, data: Vec<u8>) -> Self {
        self.with_data(data)
    }

    #[must_use]
    pub fn with_dense(mut self, dense: Denseness) -> Self {
        self.dense = dense;
        self
    }

    #[deprecated(since = "1.0.0", note = "use `with_dense` instead")]
    #[doc(hidden)]
    pub fn dense(self, is_dense: bool) -> Self {
        self.with_dense(if is_dense {
            Denseness::Dense
        } else {
            Denseness::Sparse
        })
    }

    /// Build the [`PointCloud2Msg`] from the builder.
    ///
    /// # Errors
    /// Returns an error if the fields are empty, the field count is not 1, the field format is invalid, the data length does not match the point step, or the field size is too large.
    pub fn build(self) -> Result<PointCloud2Msg, ConversionError> {
        if self.fields.is_empty() {
            return Err(ConversionError::FieldsNotFound(vec![]));
        }

        if self.fields.iter().any(|f| f.count != 1) {
            return Err(ConversionError::UnsupportedFieldCount);
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
            return Err(ConversionError::InvalidFieldFormat);
        }

        if !(self.data.len() as u32).is_multiple_of(self.point_step) {
            return Err(ConversionError::DataLengthMismatch);
        }

        Ok(PointCloud2Msg {
            header: self.header,
            dimensions: CloudDimensionsBuilder::new_with_width(self.width as usize).build()?,
            fields: self.fields,
            endian: self.endian,
            point_step: self.point_step,
            row_step: self.row_step,
            data: self.data,
            dense: self.dense,
        })
    }
}

/// Dimensions of the point cloud as width and height.
#[derive(Clone, Debug, Default, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub struct CloudDimensions {
    pub width: u32,
    pub height: u32,
}

impl CloudDimensions {
    /// Total number of points in the cloud.
    #[inline]
    pub fn len(&self) -> usize {
        (self.width as usize) * (self.height as usize)
    }

    /// Check if the cloud is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

fn ordered_field_names_from_layout(layout: &LayoutDescription) -> Vec<&'static str> {
    layout
        .0
        .iter()
        .filter(|field| matches!(field, LayoutField::Field { .. }))
        .map(|field| match field {
            LayoutField::Field { name, .. } => *name,
            _ => unreachable!("Fields must be filtered before."),
        })
        .collect()
}

impl PointCloud2Msg {
    #[inline]
    fn byte_similarity<const N: usize, C>(&self) -> Result<ByteSimilarity, ConversionError>
    where
        C: PointConvertible<N>,
    {
        let layout = C::layout();
        let field_names = ordered_field_names_from_layout(&layout);
        let target_layout = KnownLayoutInfo::try_from(layout.clone())?;

        debug_assert!(field_names.len() <= target_layout.fields.len());
        debug_assert!(self.fields.len() >= field_names.len());

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
                        return Err(ConversionError::ExhaustedSource);
                    }

                    // SAFETY: We just checked that `field_counter` is strictly less than both
                    // `self.fields.len()` and `field_names.len()`, so indexing with `get_unchecked`
                    // is within bounds.
                    let msg_f = unsafe { self.fields.get_unchecked(field_counter) };
                    // SAFETY: `field_names` is derived from the target type layout and the
                    // same index validity check above applies.
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

    #[inline]
    fn message_template_for_type<const N: usize, C>(
    ) -> Result<(PointCloud2MsgBuilder, usize), ConversionError>
    where
        C: PointConvertible<N>,
    {
        let layout = C::layout();
        let field_names = ordered_field_names_from_layout(&layout);
        debug_assert!(field_names.len() == N);

        let layout = KnownLayoutInfo::try_from(C::layout())?;
        debug_assert!(field_names.len() <= layout.fields.len());

        let mut offset: usize = 0;
        let mut fields: Vec<PointFieldMsg> = Vec::with_capacity(field_names.len());

        for f in layout.fields.into_iter() {
            match f {
                PointField::Field {
                    datatype,
                    size,
                    count,
                } => {
                    fields.push(PointFieldMsg {
                        name: crate::ros::make_field_name(field_names[fields.len()]),
                        offset: offset as u32,
                        datatype,
                        ..Default::default()
                    });
                    offset += (size * count) as usize;
                }
                PointField::Padding(size) => {
                    offset += size as usize;
                }
            }
        }

        Ok((
            PointCloud2MsgBuilder::new()
                .with_fields(fields)
                .with_point_step(offset as u32),
            offset,
        ))
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
    // let msg_out = PointCloud2Msg::try_from_iter(&cloud_points).unwrap();
    /// ```
    pub fn try_from_iter<'a, const N: usize, C>(
        iterable: impl IntoIterator<Item = &'a C>,
    ) -> Result<Self, ConversionError>
    where
        C: PointConvertible<N> + 'a,
    {
        let (mut cloud, point_step) = {
            let point: IPoint<N> = C::default().into();
            debug_assert!(point.fields.len() == N);

            let layout = C::layout();
            let field_names = crate::ordered_field_names_from_layout(&layout);
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
                    name: crate::ros::make_field_name(field_name),
                    offset: pdata_offsets_acc,
                    datatype: datatype_code,
                    count: 1,
                };

                pdata_offsets_acc += field_count * pdata_entry.datatype.size() as u32;
            }

            (
                PointCloud2MsgBuilder::new()
                    .with_fields(fields)
                    .with_point_step(pdata_offsets_acc),
                pdata_offsets_acc,
            )
        };
        let mut cloud_width = 0;

        iterable.into_iter().for_each(|pointdata| {
            let point: IPoint<N> = (*pointdata).into();

            point.fields.iter().for_each(|pdata| {
                // SAFETY: `pdata.bytes` is a fixed-size (8-byte) buffer and
                // `pdata.datatype.size()` returns the actual size of the stored datatype
                // (<= 8). Creating a subslice of that length is therefore safe.
                let truncated_bytes = unsafe {
                    core::slice::from_raw_parts(pdata.bytes.as_ptr(), pdata.datatype.size())
                };
                cloud.data.extend_from_slice(truncated_bytes);
            });

            cloud_width += 1;
        });

        cloud = cloud.with_width(cloud_width);
        cloud = cloud.with_row_step(cloud_width * point_step);

        cloud.build()
    }

    /// Create a PointCloud2Msg from a parallel iterator. Requires the `rayon` and `derive` feature to be enabled.
    #[cfg(feature = "rayon")]
    #[cfg_attr(docsrs, doc(cfg(feature = "rayon")))]
    pub fn try_from_par_iter<const N: usize, C>(
        iterable: impl rayon::iter::ParallelIterator<Item = C>,
    ) -> Result<Self, ConversionError>
    where
        C: PointConvertible<N> + Send + Sync,
    {
        Self::try_from_slice(&iterable.collect::<Vec<_>>())
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
    /// let msg_out = PointCloud2Msg::try_from_slice(&cloud_points).unwrap();
    /// ```
    ///
    /// # Errors
    /// Returns an error if the byte buffer does not match the expected layout or the message contains other discrepancies.
    pub fn try_from_slice<const N: usize, C>(slice: &[C]) -> Result<Self, ConversionError>
    where
        C: PointConvertible<N>,
    {
        match (system_endian(), Endian::default()) {
            (Endian::Big, Endian::Big) | (Endian::Little, Endian::Little) => {
                let (mut cloud, point_step) = {
                    let point: IPoint<N> = C::default().into();
                    debug_assert!(point.fields.len() == N);

                    let layout = C::layout();
                    let field_names = crate::ordered_field_names_from_layout(&layout);
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
                                    name: crate::ros::make_field_name(field_names[fields.len()]),
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
                            .with_fields(fields)
                            .with_point_step(offset),
                        offset,
                    )
                };

                let bytes_total = slice.len() * point_step as usize;
                cloud.data.resize(bytes_total, u8::default());
                let raw_data: *mut C = cloud.data.as_mut_ptr() as *mut C;

                // SAFETY: `cloud.data` was resized to `bytes_total` above, so the destination
                // pointer is valid for `bytes_total` bytes. `slice` points to `slice.len()` elements
                // of `C` and `bytes_total == slice.len() * point_step`, so copying `bytes_total`
                // bytes from `slice` into `cloud.data` is safe and non-overlapping.
                unsafe {
                    core::ptr::copy_nonoverlapping(
                        slice.as_ptr().cast::<u8>(),
                        raw_data.cast::<u8>(),
                        bytes_total,
                    );
                }

                Ok(cloud
                    .with_width(slice.len() as u32)
                    .with_row_step(slice.len() as u32 * point_step)
                    .build()?)
            }
            _ => Self::try_from_iter(slice.iter()),
        }
    }

    fn try_from_vec_strict_consuming<const N: usize, C>(
        mut vec: Vec<C>,
    ) -> Result<Self, (ConversionError, Vec<C>)>
    where
        C: PointConvertible<N> + Copy,
    {
        let sys_endian = system_endian();
        let (cloud, point_step) = match Self::message_template_for_type::<N, C>() {
            Ok(v) => v,
            Err(e) => return Err((e, vec)),
        };

        let c_size = core::mem::size_of::<C>();
        let vec_len = vec.len();
        if c_size != point_step {
            return Err((
                ConversionError::VecElementSizeMismatch {
                    element_size: c_size,
                    expected_point_step: point_step,
                },
                vec,
            ));
        }

        let bytes_total = vec_len * point_step;
        let cap_bytes = vec.capacity() * point_step;
        let ptr = vec.as_mut_ptr() as *mut u8;

        // Move ownership of the allocation from `vec` into a `Vec<u8>` without copying.
        // SAFETY: `ptr` came from the `vec` allocation and `bytes_total` is the number of
        // bytes actually used by the elements (vec.len() * point_step). `cap_bytes` is the
        // original capacity in bytes. `Vec::from_raw_parts` therefore reconstructs a valid `Vec<u8>` that owns the same allocation we just
        // forgot. After this point we must not use the original `vec` value.
        core::mem::forget(vec);
        let data = unsafe { Vec::from_raw_parts(ptr, bytes_total, cap_bytes) };

        match cloud
            .with_endian(sys_endian)
            .with_data(data)
            .with_width(vec_len as u32)
            .with_row_step((vec_len as u32) * (point_step as u32))
            .build()
        {
            Ok(msg) => Ok(msg),
            Err(_) => {
                unreachable!("The conversion should succeed since the layout matches exactly.")
            }
        }
    }

    /// Create a [`PointCloud2Msg`] from a Vec of points, trying to reuse the allocation when possible.
    /// Since the point type is known at compile time, the conversion is done by direct copy when possible.
    /// Otherwise falls back to per-point conversion.
    ///
    /// # Example
    /// ```
    /// use ros_pointcloud2::prelude::*;
    ///
    /// let cloud_points: Vec<PointXYZ> = vec![
    ///   PointXYZ::new(1.0, 2.0, 3.0),
    ///   PointXYZ::new(4.0, 5.0, 6.0),
    /// ];
    ///
    /// let msg_out = PointCloud2Msg::try_from_vec(cloud_points).unwrap();
    /// ```
    // Internal helper that attempts the strict owned conversion and returns the original `Vec<C>`
    // on failure so callers can fall back to safe constructions.
    pub fn try_from_vec_strict<const N: usize, C>(vec: Vec<C>) -> Result<Self, ConversionError>
    where
        C: PointConvertible<N> + Copy,
    {
        match Self::try_from_vec_strict_consuming(vec) {
            Ok(msg) => Ok(msg),
            Err((e, _)) => Err(e),
        }
    }

    /// Convert the [`PointCloud2Msg`] to a Vec of points, trying to reuse the allocation when possible and falling back to minimal copy.
    ///
    /// # Example
    /// ```
    /// use ros_pointcloud2::prelude::*;
    ///
    /// let cloud_points: Vec<PointXYZ> = vec![
    ///   PointXYZ::new(1.0, 2.0, 3.0),
    ///   PointXYZ::new(4.0, 5.0, 6.0),
    /// ];
    ///
    /// let msg_out = PointCloud2Msg::try_from_vec(cloud_points).unwrap();
    /// ```
    /// # Errors
    /// Returns an error if the byte buffer does not match the expected layout or the message contains
    /// other discrepancies.
    pub fn try_from_vec<const N: usize, C>(vec: Vec<C>) -> Result<Self, ConversionError>
    where
        C: PointConvertible<N> + Copy,
    {
        if let Ok((_, point_step)) = Self::message_template_for_type::<N, C>() {
            let c_size = core::mem::size_of::<C>();
            if c_size == point_step {
                match Self::try_from_vec_strict_consuming(vec) {
                    Ok(msg) => return Ok(msg),
                    Err((_, returned_vec)) => return Self::try_from_slice(&returned_vec),
                }
            }
        }

        // Fall back to minimal copy
        Self::try_from_slice(&vec)
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
    /// let msg_out = PointCloud2Msg::try_from_slice(&cloud_points).unwrap();
    /// let cloud_points_out: Vec<PointXYZ> = msg_out.try_into_vec().unwrap();
    /// assert_eq!(1.0, cloud_points_out.get(0).unwrap().x);
    /// ```
    ///
    /// # Errors
    /// Returns an error if the byte buffer does not match the expected layout or the message contains other discrepancies.
    ///
    /// **Tip:** prefer `try_into_slice` (or `try_into_slice_strict`) when you want a
    /// zero-copy view with a safe fallback to an owned `Vec<C>`; use `try_into_vec` when
    /// you always need an owned `Vec<C>`.
    pub fn try_into_vec<const N: usize, C>(&self) -> Result<Vec<C>, ConversionError>
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

                let cloud_len = self.dimensions.len();
                let point_step = self.point_step as usize;
                let mut vec: Vec<C> = Vec::with_capacity(cloud_len);
                if bytematch {
                    // SAFETY: `bytematch == true` means the message layout is byte-compatible
                    // with `C` (`byte_similarity == Equal`) and `point_step == size_of::<C>()`.
                    // Therefore `self.data.len() == cloud_len * size_of::<C>()`, which fits inside
                    // the allocated bytes for `Vec<C>` with capacity `cloud_len`. Copying the raw
                    // bytes into `vec`'s allocation and then setting its length to `cloud_len` is
                    // therefore safe and yields a valid `Vec<C>` with the correct elements.
                    unsafe {
                        core::ptr::copy_nonoverlapping(
                            self.data.as_ptr(),
                            vec.as_mut_ptr().cast::<u8>(),
                            self.data.len(),
                        );
                        vec.set_len(cloud_len);
                    }
                } else {
                    // SAFETY: fallback path reads one point at a time from the message buffer.
                    // We compute each point's byte offset as `i * point_step` and cast it to `C`.
                    // The earlier `byte_similarity` check guarantees that each field can be
                    // interpreted as the corresponding field of `C`. Reading the `C` value via
                    // `read()` is safe under those conditions.
                    unsafe {
                        for i in 0..cloud_len {
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

    /// Strict: attempt to view the message data as a zero-copy slice of `C`.
    ///
    /// This requires:
    /// - endianness matches system endianness
    /// - the field layout is byte-compatible (`byte_similarity == Equal`)
    /// - `point_step == size_of::<C>()` (no interleaving)
    /// - the underlying buffer pointer is properly aligned for `C`
    pub fn try_into_slice_strict<const N: usize, C>(&self) -> Result<&[C], ConversionError>
    where
        C: PointConvertible<N> + Copy,
    {
        if system_endian() != self.endian {
            return Err(ConversionError::UnsupportedSliceView);
        }

        // Layout must match exactly for a direct view
        if self.byte_similarity::<N, C>()? != ByteSimilarity::Equal {
            return Err(ConversionError::UnsupportedSliceView);
        }

        let c_size = core::mem::size_of::<C>();
        let point_step = self.point_step as usize;
        if point_step != c_size {
            return Err(ConversionError::UnsupportedSliceView);
        }

        if !self.data.len().is_multiple_of(c_size) {
            return Err(ConversionError::DataLengthMismatch);
        }

        let ptr = self.data.as_ptr() as *const C;
        if !(ptr as usize).is_multiple_of(core::mem::align_of::<C>()) {
            return Err(ConversionError::UnalignedBuffer);
        }

        let len = self.data.len() / c_size;
        // SAFETY: At this point we have verified:
        // - `system_endian() == self.endian`
        // - `byte_similarity::<N, C>() == Equal`
        // - `point_step == size_of::<C>()`
        // - `self.data.len()` is a multiple of `size_of::<C>()`
        // - the data pointer is properly aligned for `C`
        // Together these guarantees ensure it is safe to construct a `&[C]` from the raw
        // pointer and length using `from_raw_parts`.
        let slice = unsafe { core::slice::from_raw_parts(ptr, len) };
        Ok(slice)
    }

    /// View the message as either a borrowed slice or an owned vec (as a `Cow<[C]>`).
    ///
    /// Prefer this API over `try_into_vec` when possible: it will return a zero-copy
    /// `Cow::Borrowed(&[C])` when the message buffer is compatible with `C`, and will
    /// fall back to `Cow::Owned(Vec<C>)` (via `try_into_vec`) when a zero-copy view
    /// cannot be created.
    ///
    /// # Examples
    /// ```
    /// use ros_pointcloud2::prelude::*;
    ///
    /// let pts = vec![PointXYZ::new(1.0, 2.0, 3.0)];
    /// let msg = PointCloud2Msg::try_from_slice(&pts).unwrap();
    ///
    /// // Prefer the strict zero-copy view when possible:
    /// let slice = msg.try_into_slice_strict::<3, PointXYZ>().unwrap();
    /// assert_eq!(slice[0].x, pts[0].x);
    ///
    /// // Convenience API that falls back to an owned Vec when needed:
    /// let cow = msg.try_into_slice::<3, PointXYZ>().unwrap();
    /// match cow {
    ///     std::borrow::Cow::Borrowed(s) => assert_eq!(s.len(), pts.len()),
    ///     std::borrow::Cow::Owned(v) => assert_eq!(v.len(), pts.len()),
    /// }
    /// ```
    pub fn try_into_slice<'a, const N: usize, C>(
        &'a self,
    ) -> Result<alloc::borrow::Cow<'a, [C]>, ConversionError>
    where
        C: PointConvertible<N> + Copy,
    {
        match self.try_into_slice_strict::<N, C>() {
            Ok(slice) => Ok(alloc::borrow::Cow::Borrowed(slice)),
            Err(_) => {
                let vec = self.try_into_vec::<N, C>()?;
                Ok(alloc::borrow::Cow::Owned(vec))
            }
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
    /// let msg_out = PointCloud2Msg::try_from_iter(&cloud_points).unwrap();
    /// let cloud_points_out = msg_out.try_into_iter().unwrap().collect::<Vec<PointXYZ>>();
    /// ```
    /// # Errors
    /// Returns an error if the byte buffer does not match the expected layout or the message contains other discrepancies.
    pub fn try_into_iter<'a, const N: usize, C>(
        &'a self,
    ) -> Result<impl Iterator<Item = C> + 'a, ConversionError>
    where
        C: PointConvertible<N> + 'a,
    {
        // Return a borrowed, zero-copy iterator that reads directly from `&self.data`.
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
    /// let msg_out = PointCloud2Msg::try_from_iter(&cloud_points).unwrap();
    /// let cloud_points_out = msg_out.try_into_par_iter().unwrap().collect::<Vec<PointXYZ>>();
    /// assert_eq!(2, cloud_points_out.len());
    /// ```
    #[cfg_attr(docsrs, doc(cfg(feature = "rayon")))]
    #[cfg(feature = "rayon")]
    pub fn try_into_par_iter<'a, const N: usize, C>(
        &'a self,
    ) -> Result<impl rayon::iter::ParallelIterator<Item = C> + 'a, ConversionError>
    where
        C: PointConvertible<N> + Send + Sync + 'a,
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
pub struct IPoint<const N: usize> {
    fields: [PointData; N],
}

impl<const N: usize> core::ops::Index<usize> for IPoint<N> {
    type Output = PointData;

    fn index(&self, index: usize) -> &Self::Output {
        &self.fields[index]
    }
}

impl<const N: usize> From<[PointData; N]> for IPoint<N> {
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
/// The derive macro supports renaming fields to match the message field names.
///
/// ```ignore
/// use ros_pointcloud2::prelude::*;
///
/// #[derive(Clone, Debug, PartialEq, Copy, Default, PointConvertible)]
/// #[repr(C, align(4))]
/// pub struct MyPointXYZL {
///     pub x: f32,
///     pub y: f32,
///     pub z: f32,
///     #[ros(remap("l"))]
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
/// impl From<MyPointXYZL> for IPoint<4> {
///     fn from(point: MyPointXYZL) -> Self {
///         [point.x.into(), point.y.into(), point.z.into(), point.label.into()].into()
///     }
/// }
///
/// impl From<IPoint<4>> for MyPointXYZL {
///     fn from(point: IPoint<4>) -> Self {
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
    From<IPoint<N>> + Into<IPoint<N>> + Default + Sized + Copy
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
    type Error = ConversionError;

    fn try_from(f: LayoutField) -> Result<Self, Self::Error> {
        match f {
            LayoutField::Field { name: _, ty, size } => {
                let typename: String = ty.to_lowercase();
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
    type Error = ConversionError;

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

    /// Runtime-checked variant of `get`.
    ///
    /// - When the `strict-type-check` feature is enabled this will return an `Err(ConversionError::TypeMismatch)`
    ///   if the stored field datatype is incompatible with the requested type.
    /// - When the feature is not enabled this behaves like `get` and returns `Ok(value)`.
    pub fn get_checked<T: FromBytes>(&self) -> Result<T, ConversionError> {
        #[cfg(feature = "strict-type-check")]
        {
            let stored = self.datatype;
            let requested = T::field_datatype();
            let compatible = stored == requested
                || (matches!(stored, FieldDatatype::RGB) && requested == FieldDatatype::F32)
                || (stored == FieldDatatype::F32 && requested == FieldDatatype::RGB);
            if !compatible {
                return Err(ConversionError::TypeMismatch { stored, requested });
            }
        }
        let val = match self.endian {
            Endian::Big => T::from_be_bytes(PointDataBuffer::new(self.bytes)),
            Endian::Little => T::from_le_bytes(PointDataBuffer::new(self.bytes)),
        };
        Ok(val)
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
    type Err = ConversionError;

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
            _ => Err(ConversionError::UnsupportedFieldType(s.into())),
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
    type Error = ConversionError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use alloc::string::ToString;
        match value {
            1 => Ok(FieldDatatype::I8),
            2 => Ok(FieldDatatype::U8),
            3 => Ok(FieldDatatype::I16),
            4 => Ok(FieldDatatype::U16),
            5 => Ok(FieldDatatype::I32),
            6 => Ok(FieldDatatype::U32),
            7 => Ok(FieldDatatype::F32),
            8 => Ok(FieldDatatype::F64),
            _ => Err(ConversionError::UnsupportedFieldType(value.to_string())),
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
    type Error = ConversionError;

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
