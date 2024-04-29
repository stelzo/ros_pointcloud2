use crate::{
    pcl_utils::*,
    Point,
    PointCloud2Msg,
    PointConvertible,
    ConversionError,
    ros_types::PointFieldMsg,
    convert::{
        FromBytes,
        FieldDatatype,
    },
};

/// Convenience type for a Writer that reads coordinates as f32. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type WriterF32<const DIM: usize, const METADIM: usize, C> =
    Writer<f32, { std::mem::size_of::<f32>() }, DIM, METADIM, C>;

/// Convenience type for a Writer that reads coordinates as f64. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type WriterF64<const DIM: usize, const METADIM: usize, C> =
    Writer<f64, { std::mem::size_of::<f64>() }, DIM, METADIM, C>;

/// Convenience type for a Writer that reads coordinates as i8. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type WriterI8<const DIM: usize, const METADIM: usize, C> =
    Writer<i8, { std::mem::size_of::<i8>() }, DIM, METADIM, C>;

/// Convenience type for a Writer that reads coordinates as i16. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type WriterI16<const DIM: usize, const METADIM: usize, C> =
    Writer<i16, { std::mem::size_of::<i16>() }, DIM, METADIM, C>;

/// Convenience type for a Writer that reads coordinates as i32. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type WriterI32<const DIM: usize, const METADIM: usize, C> =
    Writer<i32, { std::mem::size_of::<i32>() }, DIM, METADIM, C>;

/// Convenience type for a Writer that reads coordinates as u8. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type WriterU8<const DIM: usize, const METADIM: usize, C> =
    Writer<u8, { std::mem::size_of::<u8>() }, DIM, METADIM, C>;

/// Convenience type for a Writer that reads coordinates as u16. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type WriterU16<const DIM: usize, const METADIM: usize, C> =
    Writer<u16, { std::mem::size_of::<u16>() }, DIM, METADIM, C>;

/// Convenience type for a Writer that reads coordinates as u32. Specify the number of dimensions, metadata dimensions and C, the point type.
pub type WriterU32<const DIM: usize, const METADIM: usize, C> =
    Writer<u32, { std::mem::size_of::<u32>() }, DIM, METADIM, C>;

/// Writes a point cloud message from an iterator over xyz coordinates (see `PointXYZ`).
pub type WriterXYZ = WriterF32<3, 0, PointXYZ>;

/// Writes a point cloud message from an iterator over xyz coordinates and intensity (see `PointXYZI`).
pub type WriterXYZI = WriterF32<3, 1, PointXYZI>;

/// Writes a point cloud message from an iterator over xyz coordinates and normal (see `PointXYZNormal`).
pub type WriterXYZNormal = WriterF32<3, 3, PointXYZNormal>;

/// Writes a point cloud message from an iterator over xyz coordinates and packs the rgb channels (see `PointXYZRGB`).
pub type WriterXYZRGB = WriterF32<3, 1, PointXYZRGB>;

/// Writes a point cloud message from an iterator over xyz coordinates and intensity and packs the rgb channels (see `PointXYZRGBL`).
pub type WriterXYZRGBL = WriterF32<3, 2, PointXYZRGBL>;

/// Writes a point cloud message from an iterator over xyz coordinates and intensity and packs the rgb channels and alpha channel (see `PointXYZRGBA`).
pub type WriterXYZRGBA = WriterF32<3, 2, PointXYZRGBA>;

/// Writes a point cloud message from an iterator over xyz coordinates and normal and packs the rgb channels (see `PointXYZRGBNormal`).
pub type WriterXYZRGBNormal = WriterF32<3, 4, PointXYZRGBNormal>;

/// Writes a point cloud message from an iterator over xyz coordinates and intensity and normal (see `PointXYZINormal`).
pub type WriterXYZINormal = WriterF32<3, 4, PointXYZINormal>;

/// Writes a point cloud message from an iterator over xyz coordinates and intensity and label (see `PointXYZL`).
pub type WriterXYZL = WriterF32<3, 1, PointXYZL>;

/// The Write creates a PointCloud2Msg out of your point iterator.
///
/// The iterator is defined at compile time, so the Point has to be described via template arguments.
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
///     pcl_utils::PointXYZ, writer::WriterXYZ, PointCloud2Msg,
/// };
///
///
/// let cloud_points: Vec<PointXYZ> = vec![
///     PointXYZ { x: 1.0, y: 2.0, z: 3.0 },
///     PointXYZ { x: 4.0, y: 5.0, z: 6.0 },
/// ];
///
/// let msg: PointCloud2Msg = WriterXYZ::from(cloud_points).try_into().unwrap();
/// //                        ^^^^^^^^^ creating PointCloud2Msg from an iterator
/// ```
/// # Fully Custom Writer
/// When the predefined types are not enough (like sensor specific metadata), you can describe your Writer with the following template arguments:
/// - T: The coordinate type, e.g. f32
/// - SIZE: The size of the coordinate type in bytes, e.g. 4 for f32. Use the ros_pointcloud2::size_of! macro for this.
/// - DIM: The dimensionality of the point, e.g. 3 for xyz coordinates.
/// - METADIM: The number of additional meta data fields per point that are not for dimensionality. Intensity for example is a meta data field.
/// Afterwards, implement the PointConvertible trait for your custom point type.
///
/// ## Example
/// ```
/// use ros_pointcloud2::{
///     reader::Reader, writer::Writer, PointConvertible, Point, size_of, MetaNames, PointMeta,
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
/// impl PointConvertible<Xyz, XYZ_S, DIM, METADIM> for CustomPoint {}
///
/// type MyReader = Reader<Xyz, XYZ_S, DIM, METADIM, CustomPoint>;
/// type MyWriter = Writer<Xyz, XYZ_S, DIM, METADIM, CustomPoint>;
/// //   ^^^^^^^^ your custom Writer
/// ```
pub struct Writer<T: FromBytes, const SIZE: usize, const DIM: usize, const METADIM: usize, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    coordinates: Box<dyn Iterator<Item = C>>,
    phantom_t: std::marker::PhantomData<T>,
}

impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> TryInto<PointCloud2Msg>
    for Writer<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    type Error = ConversionError;

    /// Writes the points to a PointCloud2Msg.
    ///
    /// First use the `from` method for initializing the Writer.
    /// Then use the `try_into` method to do the actual conversion.
    ///
    /// The operation is O(n) in time complexity where n is the number of points in the point cloud.
    ///
    /// # Example
    /// ```
    /// use ros_pointcloud2::{
    ///     pcl_utils::PointXYZ, writer::WriterXYZ, PointCloud2Msg,
    /// };
    ///
    /// let cloud_points: Vec<PointXYZ> = vec![
    ///     PointXYZ { x: 1.0, y: 2.0, z: 3.0 },
    ///     PointXYZ { x: 4.0, y: 5.0, z: 6.0 },
    /// ];
    /// let msg_out: PointCloud2Msg = WriterXYZ::from(cloud_points).try_into().unwrap();
    /// //                                                          ^^^^^^^^ ROS message conversion
    /// ```
    fn try_into(mut self) -> Result<PointCloud2Msg, Self::Error> {
        if DIM > 3 {
            return Err(ConversionError::TooManyDimensions); // maybe can be checked at compile time?
        }

        let mut fields = Vec::with_capacity(METADIM + DIM); // TODO check if we can preallocate the size on the stack

        if DIM > 1 {
            fields.push(PointFieldMsg {
                name: "x".into(),
                offset: 0,
                datatype: T::field_datatype().into(),
                count: 1,
            });

            fields.push(PointFieldMsg {
                name: "y".into(),
                offset: SIZE as u32,
                datatype: T::field_datatype().into(),
                count: 1,
            });
        }

        if DIM == 3 {
            fields.push(PointFieldMsg {
                name: "z".into(),
                offset: 2 * SIZE as u32,
                datatype: T::field_datatype().into(),
                count: 1,
            });
        }

        let first_point = self.coordinates.next().ok_or(ConversionError::NoPoints)?;
        let point: Point<T, DIM, METADIM> = first_point.clone().into();
        let meta_names = C::meta_names();

        let mut meta_offsets_acc = DIM as u32 * SIZE as u32;
        for (meta_value, meta_name) in point.meta.into_iter().zip(meta_names.into_iter()) {
            let datatype_code = meta_value.datatype.into();
            if FieldDatatype::try_from(datatype_code).is_err() {
                return Err(ConversionError::UnsupportedFieldType);
            }

            fields.push(PointFieldMsg {
                name: meta_name.into(),
                offset: meta_offsets_acc,
                datatype: datatype_code,
                count: 1,
            });
            meta_offsets_acc += meta_value.datatype.size() as u32
        }

        let mut cloud = PointCloud2Msg {
            point_step: fields.iter().fold(Default::default(), |acc, field| {
                let field_type: FieldDatatype = field
                    .datatype
                    .try_into()
                    .expect("Unsupported field but checked before.");
                let field_size = field_type.size();
                acc + field.count * field_size as u32
            }),
            ..Default::default()
        };

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
    /// Create a Writer from any iterator that iterates over a template-defined point to a ROS message type.
    /// First use the `from` method for initializing the Writer.
    /// Then use the `try_into` method to do the actual conversion.
    ///
    /// The operation is O(n) in time complexity where n is the number of points in the point cloud.
    ///
    /// # Example
    /// ```
    /// use ros_pointcloud2::{
    ///     pcl_utils::PointXYZ, writer::WriterXYZ, PointCloud2Msg,
    /// };
    ///
    /// let cloud_points: Vec<PointXYZ> = vec![
    ///     PointXYZ { x: 1.0, y: 2.0, z: 3.0 },
    ///     PointXYZ { x: 4.0, y: 5.0, z: 6.0 },
    /// ];
    // let msg_out: PointCloud2Msg = WriterXYZ::from(cloud_points).try_into().unwrap();
    /// //                                       ^^^^ Writer creation
    /// ```
    pub fn from(coordinates: impl IntoIterator<Item = C> + 'static) -> Self {
        Self {
            coordinates: Box::new(coordinates.into_iter()),
            phantom_t: std::marker::PhantomData,
        }
    }
}

#[inline(always)]
fn add_point_to_byte_buffer<
    T: FromBytes,
    const SIZE: usize,
    const DIM: usize,
    const METADIM: usize,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
>(
    coords: C,
    cloud: &mut PointCloud2Msg,
) -> Result<bool, ConversionError> {
    let point: Point<T, DIM, METADIM> = coords.into();

    // (x, y, z...)
    point
        .coords
        .iter()
        .for_each(|x| cloud.data.extend_from_slice(T::bytes(x).as_slice()));

    // meta data description
    point.meta.iter().for_each(|meta| {
        let truncated_bytes = &meta.bytes[0..meta.datatype.size()];
        cloud.data.extend_from_slice(truncated_bytes);
    });

    cloud.width += 1;

    Ok(true)
}