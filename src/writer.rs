use crate::*;

/// Writes a point cloud message from an iterator over xyz coordinates (see `PointXYZ`).
pub type WriterXYZ = Writer<f32, { size_of!(f32) }, 3, 0, PointXYZ>;

/// Writes a point cloud message from an iterator over xyz coordinates and intensity (see `PointXYZI`).
pub type WriterXYZI = Writer<f32, { size_of!(f32) }, 3, 1, PointXYZI>;

/// Writes a point cloud message from an iterator over xyz coordinates and normal (see `PointXYZNormal`).
pub type WriterXYZNormal = Writer<f32, { size_of!(f32) }, 3, 3, PointXYZNormal>;

/// Writes a point cloud message from an iterator over xyz coordinates and packs the rgb channels (see `PointXYZRGB`).
pub type WriterXYZRGB = Writer<f32, { size_of!(f32) }, 3, 1, PointXYZRGB>;

/// Writes a point cloud message from an iterator over xyz coordinates and intensity and packs the rgb channels (see `PointXYZRGBL`).
pub type WriterXYZRGBL = Writer<f32, { size_of!(f32) }, 3, 2, PointXYZRGBL>;

/// Writes a point cloud message from an iterator over xyz coordinates and intensity and packs the rgb channels and alpha channel (see `PointXYZRGBA`).
pub type WriterXYZRGBA = Writer<f32, { size_of!(f32) }, 3, 2, PointXYZRGBA>;

/// Writes a point cloud message from an iterator over xyz coordinates and normal and packs the rgb channels (see `PointXYZRGBNormal`).
pub type WriterXYZRGBNormal = Writer<f32, { size_of!(f32) }, 3, 4, PointXYZRGBNormal>;

/// Writes a point cloud message from an iterator over xyz coordinates and intensity and normal (see `PointXYZINormal`).
pub type WriterXYZINormal = Writer<f32, { size_of!(f32) }, 3, 4, PointXYZINormal>;

/// Writes a point cloud message from an iterator over xyz coordinates and intensity and label (see `PointXYZL`).
pub type WriterXYZL = Writer<f32, { size_of!(f32) }, 3, 1, PointXYZL>;

// eats an iterator, can only write from iterators to messages, so must not implement iterator pattern!
pub struct Writer<T: FromBytes, const SIZE: usize, const DIM: usize, const METADIM: usize, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    coordinates: Box<dyn Iterator<Item = C>>, // internal iterator
    xyz_field_type: FieldDatatype,
    phantom_t: std::marker::PhantomData<T>,
}

impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> TryInto<PointCloud2Msg>
    for Writer<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    type Error = ConversionError;

    /// Convert the point cloud to a ROS message.
    /// First use the `try_from` method for initializing the conversion and parsing meta data.
    /// Then use the `try_into` method to do the actual conversion.
    ///
    /// # Example
    /// ```
    /// use ros_pointcloud2::ros_types::PointCloud2Msg;
    /// use ros_pointcloud2::ConvertXYZ;
    /// use ros_pointcloud2::pcl_utils::PointXYZ;
    ///
    /// let cloud_points: Vec<PointXYZ> = vec![
    ///     PointXYZ { x: 1.0, y: 2.0, z: 3.0 },
    ///     PointXYZ { x: 4.0, y: 5.0, z: 6.0 },
    /// ];
    /// let msg_out: PointCloud2Msg = WriterXYZ::try_from(cloud_points).unwrap().try_into().unwrap();
    /// ```
    fn try_into(mut self) -> Result<PointCloud2Msg, Self::Error> {
        if DIM > 3 {
            return Err(ConversionError::TooManyDimensions); // maybe can be checked at compile time?
        }

        let datatype: u8 = convert_to_msg_code(&self.xyz_field_type);
        let mut fields = Vec::with_capacity(METADIM + DIM); // TODO check if we can preallocate the size on the stack

        if DIM > 1 {
            fields.push(PointFieldMsg {
                name: "x".to_string(),
                offset: 0,
                datatype,
                count: 1,
            });
            fields.push(PointFieldMsg {
                name: "y".to_string(),
                offset: SIZE as u32,
                datatype,
                count: 1,
            });
        }

        if DIM == 3 {
            fields.push(PointFieldMsg {
                name: "z".to_string(),
                offset: 2 * SIZE as u32,
                datatype,
                count: 1,
            });
        }

        let mut cloud = PointCloud2Msg::default();

        // self was created using a point iterator
        let first_point = self.coordinates.next().ok_or(ConversionError::NoPoints)?;
        let meta: [PointMeta; METADIM] = match first_point.clone().try_into() {
            Ok(meta) => meta.1,
            Err(_) => return Err(ConversionError::PointConversionError),
        };

        let meta_names = C::meta_names();

        let sized_dim = DIM as u32 * SIZE as u32;

        for (idx, value) in meta.iter().enumerate() {
            let datatype: u8 = convert_to_msg_code(&value.datatype);
            let mut offset = sized_dim;
            for i in 0..idx {
                let datatype: u8 = convert_to_msg_code(&meta[i].datatype);
                let field_type = convert_msg_code_to_type(datatype)?;
                let field_size = datatype_size(&field_type);
                offset += field_size as u32;
            }
            fields.push(PointFieldMsg {
                name: meta_names
                    .get(idx)
                    .ok_or(ConversionError::MetaIndexLengthMismatch)?
                    .clone(),
                offset,
                datatype,
                count: 1,
            });
        }

        // calculate point size inside byte buffer
        cloud.point_step = fields.iter().fold(0, |acc, field| {
            let field_type = convert_msg_code_to_type(field.datatype).unwrap();
            let field_size = datatype_size(&field_type);
            acc + field.count * field_size as u32
        });

        // actual point -> byte conversion -- O(n)
        add_point_to_byte_buffer(first_point, &mut cloud)?;
        for coords in self.coordinates {
            add_point_to_byte_buffer(coords, &mut cloud)?;
        }

        cloud.fields = fields;
        cloud.height = 1; // everything is in one row (unstructured)
        cloud.is_bigendian = false; // ROS default
        cloud.is_dense = true; // ROS default
        cloud.row_step = cloud.width * cloud.point_step; // Note: redundant but defined in PointCloud2 message

        Ok(cloud)
    }
}

impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C>
    Writer<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    /// Create a new Writer struct from an iterator.
    ///
    /// # Example
    /// ```
    /// ```
    pub fn from(coordinates: impl IntoIterator<Item = C> + 'static) -> Self {
        Self {
            coordinates: Box::new(coordinates.into_iter()),
            xyz_field_type: FieldDatatype::F32,
            phantom_t: std::marker::PhantomData,
        }
    }
}
