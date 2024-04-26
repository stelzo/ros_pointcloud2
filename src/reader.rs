use crate::*;

/// Provides the message as an Iterator over xyz coordinates (see `PointXYZ`).
/// Every additional meta data is ignored.
pub type ReaderXYZ = Reader<f32, { size_of!(f32) }, 3, 0, PointXYZ>;

/// Provides the message as an Iterator over xyz coordinates and intensity (see `PointXYZI`).
/// Every additional meta data is ignored.
pub type ReaderXYZI = Reader<f32, { size_of!(f32) }, 3, 1, PointXYZI>;

/// Provides the message as an Iterator over xyz coordinates and normal (see `PointXYZNormal`).
/// Every additional meta data is ignored.
pub type ReaderXYZNormal = Reader<f32, { size_of!(f32) }, 3, 3, PointXYZNormal>;

/// Provides the message as an Iterator over xyz coordinates and unpacked rgb (see `PointXYZRGB`).
/// Inside the message, the rgb field gets packed into a single u32 which is ROS standard.
/// Every additional meta data is ignored.
pub type ReaderXYZRGB = Reader<f32, { size_of!(f32) }, 3, 1, PointXYZRGB>;

/// Provides the message as an Iterator over xyz coordinates and unpacked rgb and intensity (see `PointXYZRGBL`).
/// Inside the message, the rgb field gets packed into a single u32 which is ROS standard.
/// Every additional meta data is ignored.
pub type ReaderXYZRGBL = Reader<f32, { size_of!(f32) }, 3, 2, PointXYZRGBL>;

/// Provides the message as an Iterator over xyz coordinates and unpacked rgb with additional alpha channel (see `PointXYZRGBA`).
/// Inside the message, the rgb field gets packed into a single u32 which is ROS standard.
/// Every additional meta data is ignored.
pub type ReaderXYZRGBA = Reader<f32, { size_of!(f32) }, 3, 2, PointXYZRGBA>;

/// Provides the message as an Iterator over xyz coordinates and normal and unpacked rgb (see `PointXYZRGBNormal`).
/// Inside the message, the rgb field gets packed into a single u32 which is ROS standard.
/// Every additional meta data is ignored.
pub type ReaderXYZRGBNormal = Reader<f32, { size_of!(f32) }, 3, 4, PointXYZRGBNormal>;

/// Provides the message as an Iterator over xyz coordinates and intensity and normal (see `PointXYZINormal`).
/// Every additional meta data is ignored.
pub type ReaderXYZINormal = Reader<f32, { size_of!(f32) }, 3, 4, PointXYZINormal>;

/// Provides the message as an Iterator over xyz coordinates and intensity with additional label (see `PointXYZL`).
/// Every additional meta data is ignored.
/// The label is typically used for semantic segmentation.
pub type ReaderXYZL = Reader<f32, { size_of!(f32) }, 3, 1, PointXYZL>;

/// The Reader provides an abstraction around PointCloud2Msg by implementing a fallible iterator around the data buffer.
///
/// The iterator is defined at compile time, so the Point has to be described via template arguments.
/// At runtime, the iterator catches read errors from the Msg type which typically only occur with corrupted data.
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
/// # Example
/// ```
/// use ros_pointcloud2::{
///     FallibleIterator, // needed for for_each and the other iterator methods
///     reader::ReaderXYZ,
///     writer::WriterXYZ,
///     PointCloud2Msg,
///     pcl_utils::PointXYZ,
/// };
///
/// let cloud_points: Vec<PointXYZ> = vec![
///     PointXYZ { x: 1.0, y: 2.0, z: 3.0 },
///     PointXYZ { x: 4.0, y: 5.0, z: 6.0 },
/// ];
///
/// let msg: PointCloud2Msg = WriterXYZ::from(cloud_points).try_into().unwrap();
/// let convert = ReaderXYZ::try_from(msg).unwrap(); // parse message
///
/// convert.for_each(|point: PointXYZ| {
///     // do something with the point
///     Ok(()) // return Ok to continue iteration
/// })
/// .unwrap(); // handle point conversion errors
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
impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> FallibleIterator
    for Reader<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    type Item = C;
    type Error = ConversionError;

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
    fn next(&mut self) -> Result<Option<Self::Item>, Self::Error> {
        if self.iteration >= self.cloud_length {
            return Ok(None); // iteration finished
        }

        let mut xyz: [T; DIM] = [T::default(); DIM];
        let mut meta: [PointMeta; METADIM] = [PointMeta::default(); METADIM];
        for (idx, in_point_offset) in self.offsets.iter().enumerate() {
            if idx < DIM {
                match load_loadable::<T, SIZE>(
                    (self.iteration * self.point_step_size) + in_point_offset,
                    &self.data,
                    &self.endianness,
                ) {
                    Some(c) => xyz[idx] = c,
                    None => return Err(ConversionError::EndOfBuffer),
                }
            } else {
                let meta_idx = idx - DIM;
                let meta_type = self.meta[meta_idx].1;
                let start = (self.iteration * self.point_step_size) + in_point_offset;
                if let Ok(m) = PointMeta::new_from_buffer(&self.data, start, &meta_type) {
                    meta[meta_idx] = m;
                } else {
                    return Err(ConversionError::MetaIndexLengthMismatch);
                }
            }
        }

        self.iteration += 1;

        match C::try_from((xyz, meta)) {
            Err(_) => Err(ConversionError::PointConversionError),
            Ok(tuple) => Ok(Some(tuple)),
        }
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
    ///     FallibleIterator, // needed for for_each and the other iterator methods
    ///     reader::ReaderXYZ,
    ///     writer::WriterXYZ,
    ///     PointCloud2Msg,
    ///     pcl_utils::PointXYZ,
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
                        println!("{} {} {}", idx, field.name, meta_with_offsets.len());
                        match meta_with_offsets.get_mut(idx - DIM) {
                            None => return Err(ConversionError::MetaIndexLengthMismatch),
                            Some(data) => {
                                data.0 = field.name.clone();
                                data.1 = convert_msg_code_to_type(field.datatype)?;
                                data.2 = field.offset as usize;
                            }
                        }
                    }
                }
            }
        }

        meta_with_offsets.sort_unstable_by(|a, b| a.2.cmp(&b.2));
        let mut meta_offsets = [usize::default(); METADIM];
        let mut meta: Vec<(String, FieldDatatype)> =
            vec![(String::default(), FieldDatatype::U8); METADIM];
        meta_with_offsets
            .into_iter()
            .enumerate()
            .for_each(|(idx, (name, datatype, offset))| {
                unsafe {
                    // all arrays have size METADIM
                    *meta.get_unchecked_mut(idx) = (name, datatype);
                    *meta_offsets.get_unchecked_mut(idx) = offset;
                };
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
        if point_step_size * cloud.width as usize * cloud.height as usize != cloud.data.len() {
            return Err(ConversionError::DataLengthMismatch);
        }

        if let Some(max_offset) = offsets.last() {
            if let Some(last_meta) = meta.last() {
                let size_with_last_meta = max_offset + datatype_size(&last_meta.1);
                if size_with_last_meta > point_step_size {
                    return Err(ConversionError::DataLengthMismatch);
                }
            }
        }

        Ok(Self {
            iteration: 0,
            data: cloud.data,
            point_step_size: point_step_size,
            cloud_length: cloud.width as usize * cloud.height as usize,
            offsets: offsets,
            meta: meta,
            endianness: endian,
            phantom_t: std::marker::PhantomData,
            phantom_c: std::marker::PhantomData,
        })
    }
}
