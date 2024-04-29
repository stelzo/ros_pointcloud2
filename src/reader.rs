use crate::*;

/// Convenience type for a Reader that reads coordinates as f32. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type ReaderF32<const DIM: usize, const METADIM: usize, C> =
    Reader<f32, { std::mem::size_of::<f32>() }, DIM, METADIM, C>;

/// Convenience type for a Reader that reads coordinates as f64. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type ReaderF64<const DIM: usize, const METADIM: usize, C> =
    Reader<f64, { std::mem::size_of::<f64>() }, DIM, METADIM, C>;

/// Convenience type for a Reader that reads coordinates as i8. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type ReaderI8<const DIM: usize, const METADIM: usize, C> =
    Reader<i8, { std::mem::size_of::<i8>() }, DIM, METADIM, C>;

/// Convenience type for a Reader that reads coordinates as i16. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type ReaderI16<const DIM: usize, const METADIM: usize, C> =
    Reader<i16, { std::mem::size_of::<i16>() }, DIM, METADIM, C>;

/// Convenience type for a Reader that reads coordinates as i32. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type ReaderI32<const DIM: usize, const METADIM: usize, C> =
    Reader<i32, { std::mem::size_of::<i32>() }, DIM, METADIM, C>;

/// Convenience type for a Reader that reads coordinates as u8. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type ReaderU8<const DIM: usize, const METADIM: usize, C> =
    Reader<u8, { std::mem::size_of::<u8>() }, DIM, METADIM, C>;

/// Convenience type for a Reader that reads coordinates as u16. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type ReaderU16<const DIM: usize, const METADIM: usize, C> =
    Reader<u16, { std::mem::size_of::<u16>() }, DIM, METADIM, C>;

/// Convenience type for a Reader that reads coordinates as u32. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type ReaderU32<const DIM: usize, const METADIM: usize, C> =
    Reader<u32, { std::mem::size_of::<u32>() }, DIM, METADIM, C>;

/// Provides the message as an Iterator over xyz coordinates (see `PointXYZ`).
/// Every additional meta data is ignored.
pub type ReaderXYZ = ReaderF32<3, 0, PointXYZ>;

/// Provides the message as an Iterator over xyz coordinates and intensity (see `PointXYZI`).
/// Every additional meta data is ignored.
pub type ReaderXYZI = ReaderF32<3, 1, PointXYZI>;

/// Provides the message as an Iterator over xyz coordinates and normal (see `PointXYZNormal`).
/// Every additional meta data is ignored.
pub type ReaderXYZNormal = ReaderF32<3, 3, PointXYZNormal>;

/// Provides the message as an Iterator over xyz coordinates and unpacked rgb (see `PointXYZRGB`).
/// Inside the message, the rgb field gets packed into a single u32 which is ROS standard.
/// Every additional meta data is ignored.
pub type ReaderXYZRGB = ReaderF32<3, 1, PointXYZRGB>;

/// Provides the message as an Iterator over xyz coordinates and unpacked rgb and intensity (see `PointXYZRGBL`).
/// Inside the message, the rgb field gets packed into a single u32 which is ROS standard.
/// Every additional meta data is ignored.
pub type ReaderXYZRGBL = ReaderF32<3, 2, PointXYZRGBL>;

/// Provides the message as an Iterator over xyz coordinates and unpacked rgb with additional alpha channel (see `PointXYZRGBA`).
/// Inside the message, the rgb field gets packed into a single u32 which is ROS standard.
/// Every additional meta data is ignored.
pub type ReaderXYZRGBA = ReaderF32<3, 2, PointXYZRGBA>;

/// Provides the message as an Iterator over xyz coordinates and normal and unpacked rgb (see `PointXYZRGBNormal`).
/// Inside the message, the rgb field gets packed into a single u32 which is ROS standard.
/// Every additional meta data is ignored.
pub type ReaderXYZRGBNormal = ReaderF32<3, 4, PointXYZRGBNormal>;

/// Provides the message as an Iterator over xyz coordinates and intensity and normal (see `PointXYZINormal`).
/// Every additional meta data is ignored.
pub type ReaderXYZINormal = ReaderF32<3, 4, PointXYZINormal>;

/// Provides the message as an Iterator over xyz coordinates and intensity with additional label (see `PointXYZL`).
/// Every additional meta data is ignored.
/// The label is typically used for semantic segmentation.
pub type ReaderXYZL = ReaderF32<3, 1, PointXYZL>;

/// The Reader provides a an iterator abstraction of the PointCloud2Msg.
///
/// The iterator is defined at compile time, so the point has to be described via template arguments.
///
/// # Predefined Readers
/// For the most common use cases, there are already predefined types. Examples are `ReaderXYZ` for xyz coordinates or `ReaderXYZI` for xyz coordinates and intensity.
///
/// When using within a ROS node, the PointCloud2 created by the ROS crate must be converted first.
/// The cost of this operation is low, as it mostly moves ownership without iterating over the point data.
///
/// ROS1 with rosrust:
/// let msg: rosrust_msg::sensor_msgs::PointCloud2; // inside the callback
/// let converted: ros_pointcloud2::PointCloud2Msg = msg.into();
///
/// ROS2 with r2r:
/// let msg: r2r::sensor_msgs::msg::PointCloud2 = internal_msg.into();
/// let converted: ros_pointcloud2::PointCloud2Msg = msg.into();
///
/// ros_pointcloud2 supports r2r, ros2_rust and rosrust as conversion targets out of the box via feature flags.
///
/// ## Example
/// ```
/// use ros_pointcloud2::{
///     pcl_utils::PointXYZ, reader::ReaderXYZ, writer::WriterXYZ, PointCloud2Msg,
/// };
///
///
/// let cloud_points: Vec<PointXYZ> = vec![
///     PointXYZ { x: 1.0, y: 2.0, z: 3.0 },
///     PointXYZ { x: 4.0, y: 5.0, z: 6.0 },
/// ];
///
/// let msg: PointCloud2Msg = WriterXYZ::from(cloud_points).try_into().unwrap();
/// let convert = ReaderXYZ::try_from(msg).unwrap();
/// //            ^^^^^^^^^ conversion from PointCloud2Msg to Reader that implements Iterator
///
/// convert.for_each(|point: PointXYZ| {
///     // do something with the point
/// });
/// ```
/// # Fully Custom Reader
/// When the predefined types are not enough (like sensor specific metadata), you can describe your Reader with the following template arguments:
/// - T: The coordinate type, e.g. f32
/// - SIZE: The size of the coordinate type in bytes, e.g. 4 for f32. Use the ros_pointcloud2::size_of! macro for this.
/// - DIM: The dimensionality of the point, e.g. 3 for xyz coordinates.
/// - METADIM: The number of additional meta data fields per point that are not for dimensionality. Intensity for example is a meta data field.
/// Afterwards, implement the PointConvertible trait for your custom point type.
///
/// ## Example
/// ```
/// use ros_pointcloud2::{
///     reader::Reader, writer::Writer, PointConvertible, Point, size_of, convert::MetaNames, PointMeta,
/// };
///
/// type Xyz = f32; // coordinate type
/// const XYZ_S: usize = size_of!(Xyz);
/// const DIM: usize = 3; // helper constant for dimensionality
/// const METADIM: usize = 1; // helper constant for meta data fields
///
/// #[derive(Debug, PartialEq, Clone)]
/// struct CustomPoint {
///     x: f32,
///     y: f32,
///     z: f32,
///     i: u8,
/// }
///
/// impl From<Point<f32, 3, 1>> for CustomPoint {
///     fn from(point: Point<f32, 3, 1>) -> Self {
///         Self {
///             x: point.coords[0],
///             y: point.coords[1],
///             z: point.coords[2],
///             i: point.meta[0].get(),
///         }
///     }
/// }
///
///impl From<CustomPoint> for Point<f32, 3, 1> {
///    fn from(point: CustomPoint) -> Self {
///        Point {
///            coords: [point.x, point.y, point.z],
///            meta: [
///                point.i.into(),
///            ],
///        }
///    }
///}
///
/// impl MetaNames<METADIM> for CustomPoint {
///     fn meta_names() -> [&'static str; METADIM] {
///         ["intensity"]
///     }
/// }
///
/// impl PointConvertible<Xyz, XYZ_S, DIM, METADIM> for CustomPoint {}
///
/// type MyReader = Reader<Xyz, XYZ_S, DIM, METADIM, CustomPoint>;
/// //   ^^^^^^^^ your custom Reader
/// type MyWriter = Writer<Xyz, XYZ_S, DIM, METADIM, CustomPoint>;
/// ```
pub struct Reader<T: FromBytes, const SIZE: usize, const DIM: usize, const METADIM: usize, C>
where
    T: FromBytes,
    C: MetaNames<METADIM>,
{
    iteration: usize,
    data: Vec<u8>,
    point_step_size: usize,
    cloud_length: usize,
    offsets: Vec<usize>,
    meta: Vec<(String, FieldDatatype)>,
    endianness: Endianness,
    phantom_t: std::marker::PhantomData<T>, // internally used for byte and datatype conversions
    phantom_c: std::marker::PhantomData<C>, // internally used for meta names array
}

/// The iterator implementation for the Reader struct.
/// The iterator is fallible because the data is read from a byte buffer inside the PointCloud2 message, which is inherently unsafe.
///
/// See ConversionError for possible errors that can occur during iteration.
impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> Iterator
    for Reader<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    type Item = C;

    /// The size_hint is the length of the remaining elements and the maximum length of the iterator.
    ///
    /// PointCloud2 messages contain the length of the cloud, so we can prepare coming operations.
    /// This hint is used inside common iterator functions like `collect<Vec<_>>`, for example.
    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.cloud_length - self.iteration, Some(self.cloud_length))
    }

    /// Get the data from the byte buffer and convert it to the predefined point.
    /// It must keep track of the current iteration and the length of the cloud so it has to mutate self.
    ///
    /// The current point is then converted into the custom type. If the conversion fails, an error is returned.
    fn next(&mut self) -> Option<Self::Item> {
        if self.iteration >= self.cloud_length {
            return None; // iteration finished
        }

        let mut xyz = [T::default(); DIM];
        xyz.iter_mut()
            .zip(self.offsets.iter())
            .for_each(|(p_xyz, in_point_offset)| {
                *p_xyz = load_loadable::<T, SIZE>(
                    (self.iteration * self.point_step_size) + in_point_offset,
                    &self.data,
                    &self.endianness,
                );
            });

        debug_assert!(self.meta.len() == METADIM, "Meta data length mismatch");
        debug_assert!(
            self.offsets.len() == DIM + METADIM,
            "Offset length mismatch"
        );

        let mut meta = [PointMeta::default(); METADIM];
        meta.iter_mut()
            .zip(self.offsets.iter().skip(DIM))
            .zip(self.meta.iter())
            .for_each(|((p_meta, in_point_offset), (_, meta_type))| {
                let start = (self.iteration * self.point_step_size) + in_point_offset;
                *p_meta = PointMeta::from_buffer(&self.data, start, meta_type);
            });

        self.iteration += 1;

        Some(C::from(Point { coords: xyz, meta }))
    }
}

impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> TryFrom<PointCloud2Msg>
    for Reader<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: MetaNames<METADIM>,
{
    type Error = ConversionError;

    /// Convert a PointCloud2Msg into a Reader.
    /// Converting a PointCloud2Msg into a Reader is a fallible operation since the message can contain only a subset of the required fields.
    ///
    /// The theoretical time complexity is O(n) where n is the number of fields defined in the message for a single point which is typically small.
    /// It therefore has effectively a constant time complexity O(1) for practical purposes.
    ///
    /// # Example
    /// ```
    /// use ros_pointcloud2::{
    ///     pcl_utils::PointXYZ, reader::ReaderXYZ, writer::WriterXYZ, PointCloud2Msg,
    /// };
    ///
    /// let cloud_points: Vec<PointXYZ> = vec![
    ///     PointXYZ { x: 1.0, y: 2.0, z: 3.0 },
    ///     PointXYZ { x: 4.0, y: 5.0, z: 6.0 },
    /// ];
    ///
    /// let msg: PointCloud2Msg = WriterXYZ::from(cloud_points).try_into().unwrap();
    ///
    /// let convert = ReaderXYZ::try_from(msg).unwrap();
    /// //                       ^^^^^^^^ conversion from PointCloud2Msg to Reader
    /// ```
    fn try_from(cloud: PointCloud2Msg) -> Result<Self, Self::Error> {
        if cloud.fields.len() < DIM {
            return Err(ConversionError::NotEnoughFields);
        }

        let xyz_field_type = T::field_datatype();

        let mut has_x: Option<usize> = None;
        let mut has_y: Option<usize> = None;
        let mut has_z: Option<usize> = None;

        let mut meta_with_offsets = vec![(String::default(), FieldDatatype::default(), 0); METADIM];

        let lower_meta_names = C::meta_names()
            .iter()
            .map(|x| x.to_lowercase())
            .collect::<Vec<String>>();

        for (idx, field) in cloud.fields.iter().enumerate() {
            let lower_field_name = field.name.to_lowercase();
            match lower_field_name.as_str() {
                "x" => has_x = Some(idx),
                "y" => has_y = Some(idx),
                "z" => has_z = Some(idx),
                _ => {
                    if lower_meta_names.contains(&lower_field_name) {
                        let meta_idx = idx - DIM;
                        debug_assert!(
                            meta_idx < meta_with_offsets.len(),
                            "Meta data length mismatch"
                        );
                        meta_with_offsets[meta_idx].0 = field.name.clone();
                        meta_with_offsets[meta_idx].1 = field.datatype.try_into()?;
                        meta_with_offsets[meta_idx].2 = field.offset as usize;
                    }
                }
            }
        }

        meta_with_offsets.sort_unstable_by(|(_, _, offset1), (_, _, offset2)| offset1.cmp(offset2));

        debug_assert!(
            meta_with_offsets.len() == METADIM,
            "Meta data length mismatch"
        );

        let mut meta_offsets = [usize::default(); METADIM];
        let mut meta = vec![(String::default(), FieldDatatype::default()); METADIM];

        meta_with_offsets
            .into_iter()
            .zip(meta.iter_mut())
            .zip(meta_offsets.iter_mut())
            .for_each(|(((name, datatype, offset), meta), meta_offset)| {
                *meta = (name, datatype);
                *meta_offset = offset;
            });

        let x_field = check_coord(has_x, &cloud.fields, &xyz_field_type)?;
        let y_field = check_coord(has_y, &cloud.fields, &xyz_field_type)?;

        let mut offsets = vec![x_field.offset as usize, y_field.offset as usize];

        let z_field = check_coord(has_z, &cloud.fields, &xyz_field_type);
        match z_field {
            Ok(z_field) => {
                offsets.push(z_field.offset as usize);
            }
            Err(err) => match err {
                ConversionError::NotEnoughFields => {
                    if DIM == 3 {
                        return Err(ConversionError::NotEnoughFields);
                    }
                }
                _ => return Err(err),
            },
        }

        let endian = if cloud.is_bigendian {
            Endianness::Big
        } else {
            Endianness::Little
        };

        if offsets.len() != DIM {
            return Err(ConversionError::NotEnoughFields);
        }

        offsets.extend(meta_offsets);

        if offsets.len() != DIM + METADIM {
            return Err(ConversionError::NotEnoughFields);
        }

        let point_step_size = cloud.point_step as usize;
        let cloud_length = cloud.width as usize * cloud.height as usize;
        if point_step_size * cloud_length != cloud.data.len() {
            return Err(ConversionError::DataLengthMismatch);
        }

        let last_offset = offsets.last().expect("Dimensionality is 0.");

        if let Some(last_meta) = meta.last() {
            let size_with_last_meta = last_offset + last_meta.1.size();
            if size_with_last_meta > point_step_size {
                return Err(ConversionError::DataLengthMismatch);
            }
        } else if last_offset + xyz_field_type.size() > point_step_size {
            return Err(ConversionError::DataLengthMismatch);
        }

        Ok(Self {
            iteration: 0,
            data: cloud.data,
            point_step_size,
            cloud_length: cloud.width as usize * cloud.height as usize,
            offsets,
            meta,
            endianness: endian,
            phantom_t: std::marker::PhantomData,
            phantom_c: std::marker::PhantomData,
        })
    }
}
