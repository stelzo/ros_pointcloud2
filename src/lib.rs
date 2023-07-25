#![doc = include_str!("../README.md")]
pub mod pcl_utils;
pub mod ros_types;

use crate::pcl_utils::*;
use crate::ros_types::{PointCloud2Msg, PointFieldMsg};
use num_traits::Zero;

#[macro_use]
pub extern crate mem_macros;

pub extern crate fallible_iterator;

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
where
    T: FromBytes,
{
}

/// Datatypes from the [PointField message](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html).
#[derive(Clone, Debug, PartialEq, Copy)]
pub enum FieldDatatype {
    F32,
    F64,
    I32,
    U8,
    U16,
    U32,
    I8,
    I16,
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

fn convert_to_msg_code(geo_type: &FieldDatatype) -> u8 {
    match geo_type {
        FieldDatatype::I8 => 1,
        FieldDatatype::U8 => 2,
        FieldDatatype::I16 => 3,
        FieldDatatype::U16 => 4,
        FieldDatatype::I32 => 5,
        FieldDatatype::U32 => 6,
        FieldDatatype::F32 => 7,
        FieldDatatype::F64 => 8,
    }
}

fn convert_msg_code_to_type(code: u8) -> Result<FieldDatatype, ConversionError> {
    match code {
        7 => Ok(FieldDatatype::F32),
        8 => Ok(FieldDatatype::F64),
        5 => Ok(FieldDatatype::I32),
        2 => Ok(FieldDatatype::U8),
        4 => Ok(FieldDatatype::U16),
        6 => Ok(FieldDatatype::U32),
        1 => Ok(FieldDatatype::I8),
        3 => Ok(FieldDatatype::I16),
        _ => Err(ConversionError::UnsupportedFieldType),
    }
}

fn check_coord(
    coord: Option<usize>,
    fields: &[PointFieldMsg],
    xyz_field_type: &FieldDatatype,
) -> Result<PointFieldMsg, ConversionError> {
    match coord {
        Some(y_idx) => {
            let field = &fields[y_idx];
            if field.datatype != convert_to_msg_code(xyz_field_type) {
                return Err(ConversionError::InvalidFieldFormat);
            }
            Ok(field.clone())
        }
        None => Err(ConversionError::NotEnoughFields),
    }
}

/// Matching field names from each meta data per point to the PointField name.
/// Always make sure to use the same order as in your Into<> implementation to have a correct mapping.
pub trait MetaNames<const METADIM: usize> {
    fn meta_names() -> [String; METADIM];
}

/// The Convert struct is used to convert between a PointCloud2 message and a custom type.
/// A custom type must implement the FromBytes trait and the Into trait.
/// The Into trait is used to convert the custom type into a tuple of the coordinates and the meta data.
/// The FromBytes trait is used to convert the bytes from the PointCloud2 message into the custom type.
///
/// # Example
/// ```
/// use ros_pointcloud2::mem_macros::size_of;
/// use ros_pointcloud2::{Convert, PointMeta};
/// use ros_pointcloud2::ros_types::PointCloud2Msg;
/// const DIM : usize = 3; // how many dimensions your point has (e.g. x, y, z)
/// const METADIM : usize = 4; // how many meta fields you have (e.g. r, g, b, a)
/// type MyConverter = Convert<f32, { size_of!(f32) }, DIM, METADIM, ([f32; DIM], [PointMeta; METADIM])>;
/// ```
pub struct Convert<T: FromBytes, const SIZE: usize, const DIM: usize, const METADIM: usize, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    iteration: usize,
    coordinates: Vec<C>,
    phantom_t: std::marker::PhantomData<T>,
    data: Vec<u8>,
    point_step_size: usize,
    cloud_length: usize,
    offsets: Vec<usize>,
    big_endian: Endianness,
    xyz_field_type: FieldDatatype,
    meta: Vec<(String, FieldDatatype)>,
}

pub type ConvertXYZ = Convert<f32, { size_of!(f32) }, 3, 0, PointXYZ>;
pub type ConvertXYZI = Convert<f32, { size_of!(f32) }, 3, 1, PointXYZI>;
pub type ConvertXYZNormal = Convert<f32, { size_of!(f32) }, 3, 3, PointXYZNormal>;
pub type ConvertXYZRGB = Convert<f32, { size_of!(f32) }, 3, 1, PointXYZRGB>;
pub type ConvertXYZRGBL = Convert<f32, { size_of!(f32) }, 3, 2, PointXYZRGBL>;
pub type ConvertXYZRGBA = Convert<f32, { size_of!(f32) }, 3, 2, PointXYZRGBA>;
pub type ConvertXYZRGBNormal = Convert<f32, { size_of!(f32) }, 3, 4, PointXYZRGBNormal>;
pub type ConvertXYZINormal = Convert<f32, { size_of!(f32) }, 3, 4, PointXYZINormal>;
pub type ConvertXYZL = Convert<f32, { size_of!(f32) }, 3, 1, PointXYZL>;

impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> TryFrom<Vec<C>>
    for Convert<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    type Error = ConversionError;

    /// Converts a vector of custom types into a Convert struct that implements the Iterator trait.
    ///
    /// # Example
    /// ```
    /// use ros_pointcloud2::{ConvertXYZ, ConversionError};
    /// use ros_pointcloud2::pcl_utils::PointXYZ;
    ///
    /// let cloud_points: Vec<PointXYZ> = vec![
    ///     PointXYZ { x: 1.0, y: 2.0, z: 3.0 },
    ///     PointXYZ { x: 4.0, y: 5.0, z: 6.0 },
    /// ];
    /// let convert: Result<ConvertXYZ, ConversionError> = ConvertXYZ::try_from(cloud_points);
    /// ```
    fn try_from(cloud: Vec<C>) -> Result<Self, Self::Error> {
        let length = cloud.len();

        let mut meta: Vec<(String, FieldDatatype)> = vec![];
        let first_point = cloud.first().ok_or(ConversionError::NoPoints)?;
        let point_meta: ([T; DIM], [PointMeta; METADIM]) = match first_point.clone().try_into() {
            Ok(point_meta) => point_meta,
            Err(_) => {
                return Err(ConversionError::PointConversionError);
            }
        };
        let meta_names = C::meta_names();
        let mut point_step_size = DIM * SIZE;
        for (idx, value) in point_meta.1.iter().enumerate() {
            meta.push((
                meta_names
                    .get(idx)
                    .ok_or(ConversionError::MetaIndexLengthMismatch)?
                    .clone(),
                value.datatype,
            ));
            point_step_size += datatype_size(&value.datatype);
        }

        Ok(Self {
            phantom_t: std::marker::PhantomData,
            iteration: usize::zero(),
            coordinates: cloud,
            data: Vec::new(),
            point_step_size,
            cloud_length: length,
            offsets: Vec::new(),
            big_endian: Endianness::Little,
            xyz_field_type: T::field_datatype(),
            meta,
        })
    }
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
    bytes: [u8; size_of!(f64)],
    datatype: FieldDatatype,
}

impl Default for PointMeta {
    fn default() -> Self {
        Self {
            bytes: [0; size_of!(f64)],
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
        let mut bytes = [0; size_of!(f64)];
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
        let mut bytes_array = [0; size_of!(f64)]; // 8 bytes as f64 is the largest type
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

impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> TryFrom<PointCloud2Msg>
    for Convert<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    type Error = ConversionError;

    /// Converts a ROS PointCloud2 message into a Convert struct that implements the Iterator trait.
    /// Iterate over the struct to get or use the points.
    ///
    /// # Example
    /// Since we do not have a ROS node here, we first create a PointCloud2 message and then convert back to a Convert struct.
    /// ```
    /// use ros_pointcloud2::ros_types::PointCloud2Msg;
    /// use ros_pointcloud2::ConvertXYZ;
    /// use ros_pointcloud2::pcl_utils::PointXYZ;
    ///
    /// let cloud_points: Vec<PointXYZ> = vec![
    ///     PointXYZ { x: 1.0, y: 2.0, z: 3.0 },
    ///     PointXYZ { x: 4.0, y: 5.0, z: 6.0 },
    /// ];
    /// let msg: PointCloud2Msg = ConvertXYZ::try_from(cloud_points).unwrap().try_into().unwrap();
    ///
    /// let convert: ConvertXYZ = ConvertXYZ::try_from(msg).unwrap(); // parse message
    /// ```
    fn try_from(cloud: PointCloud2Msg) -> Result<Self, Self::Error> {
        if cloud.fields.len() < DIM {
            return Err(ConversionError::NotEnoughFields);
        }

        let xyz_field_type = T::field_datatype();

        let mut has_x: Option<usize> = None;
        let mut has_y: Option<usize> = None;
        let mut has_z: Option<usize> = None;

        let mut meta_with_offsets: Vec<(String, FieldDatatype, usize)> =
            Vec::with_capacity(METADIM);

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
                        meta_with_offsets.push((
                            field.name.clone(),
                            convert_msg_code_to_type(field.datatype)?,
                            field.offset as usize,
                        ));
                    }
                }
            }
        }

        meta_with_offsets.sort_by(|a, b| a.2.cmp(&b.2));
        let meta_offsets: Vec<usize> = meta_with_offsets.iter().map(|x| x.2).collect();
        let meta: Vec<(String, FieldDatatype)> = meta_with_offsets
            .iter()
            .map(|x| (x.0.clone(), x.1))
            .collect();

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
            phantom_t: std::marker::PhantomData,
            iteration: usize::zero(),
            coordinates: Vec::new(),
            data: cloud.data,
            point_step_size,
            cloud_length: cloud.width as usize * cloud.height as usize,
            offsets,
            big_endian: endian,
            xyz_field_type: T::field_datatype(),
            meta,
        })
    }
}

fn datatype_size(datatype: &FieldDatatype) -> usize {
    match datatype {
        FieldDatatype::U8 => 1,
        FieldDatatype::U16 => 2,
        FieldDatatype::U32 => 4,
        FieldDatatype::I8 => 1,
        FieldDatatype::I16 => 2,
        FieldDatatype::I32 => 4,
        FieldDatatype::F32 => 4,
        FieldDatatype::F64 => 8,
    }
}

impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> TryInto<PointCloud2Msg>
    for Convert<T, SIZE, DIM, METADIM, C>
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
    /// let msg_out: PointCloud2Msg = ConvertXYZ::try_from(cloud_points).unwrap().try_into().unwrap();
    /// ```
    fn try_into(self) -> Result<PointCloud2Msg, Self::Error> {
        let mut cloud = PointCloud2Msg::default();

        // Define the message fields
        let mut fields = Vec::new();
        if DIM > 3 {
            return Err(ConversionError::TooManyDimensions);
        }

        let datatype: u8 = convert_to_msg_code(&self.xyz_field_type);

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

        // meta data fields
        let first_point = self.coordinates.first().ok_or(ConversionError::NoPoints)?;
        let meta: ([T; DIM], [PointMeta; METADIM]) = match first_point.clone().try_into() {
            Ok(meta) => meta,
            Err(_) => return Err(ConversionError::PointConversionError),
        };

        let meta_names = C::meta_names();

        let sized_dim = DIM as u32 * SIZE as u32;

        for (idx, value) in meta.1.iter().enumerate() {
            let datatype: u8 = convert_to_msg_code(&value.datatype);
            let mut offset = sized_dim;
            for i in 0..idx {
                let datatype: u8 = convert_to_msg_code(&meta.1[i].datatype);
                let field_type = convert_msg_code_to_type(datatype)?;
                let field_size = datatype_size(&field_type);
                offset += field_size as u32;
            }
            fields.push(PointFieldMsg {
                name: meta_names[idx].to_string(),
                offset,
                datatype,
                count: 1,
            });
        }

        // calc all meta data points step size
        let mut step_size = 0;
        for field in fields.iter() {
            let field_type = convert_msg_code_to_type(field.datatype)?;
            let field_size = datatype_size(&field_type);
            step_size += field.count * field_size as u32;
        }

        cloud.fields = fields;
        cloud.point_step = step_size;
        cloud.row_step = self.coordinates.len() as u32 * cloud.point_step;
        cloud.height = 1;
        cloud.width = self.coordinates.len() as u32;
        cloud.is_bigendian = false;
        cloud.is_dense = true;

        for coords in self.coordinates {
            let coords_data: ([T; DIM], [PointMeta; METADIM]) = match coords.try_into() {
                Ok(meta) => meta,
                Err(_) => return Err(ConversionError::PointConversionError),
            };
            coords_data
                .0
                .iter()
                .for_each(|x| cloud.data.extend_from_slice(T::bytes(x).as_slice()));
            coords_data.1.iter().for_each(|meta| {
                let truncated_bytes = &meta.bytes[0..datatype_size(&meta.datatype)];
                cloud.data.extend_from_slice(truncated_bytes);
            });
        }

        Ok(cloud)
    }
}

impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C>
    Convert<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    /// Convenience getter for the number of points in the cloud.
    pub fn len(&self) -> usize {
        self.cloud_length
    }

    pub fn is_empty(&self) -> bool {
        self.cloud_length == 0
    }
}

impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C>
    fallible_iterator::FallibleIterator for Convert<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
{
    type Item = C;
    type Error = ConversionError;

    /// Iterate over the data buffer and interpret the data as a point.
    fn next(&mut self) -> Result<Option<Self::Item>, Self::Error> {
        if self.iteration >= self.cloud_length {
            return Ok(None); // iteration finished
        }

        let mut xyz: [T; DIM] = [T::zero(); DIM];
        let mut meta: [PointMeta; METADIM] = [PointMeta::default(); METADIM];
        for (idx, in_point_offset) in self.offsets.iter().enumerate() {
            if idx < DIM {
                match load_loadable::<T, SIZE>(
                    (self.iteration * self.point_step_size) + in_point_offset,
                    &self.data,
                    self.big_endian.clone(),
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
                    return Err(ConversionError::MetaIndexLengthMismatch); // since we can not return an error in the iterator, we finish the iteration here early since the last point is not valid. This case is not expected since we catch it while parsing the file.
                }
            }
        }

        self.iteration += 1;
        let conv = C::try_from((xyz, meta)); // try convert the point
        match conv {
            Err(_) => Err(ConversionError::PointConversionError),
            Ok(tuple) => Ok(Some(tuple)),
        }
    }
}

/// This trait is used to convert a byte slice to a primitive type.
/// All PointField types are supported.
pub trait FromBytes: Zero + Sized + Copy + GetFieldDatatype {
    fn from_be_bytes(bytes: &[u8]) -> Self;
    fn from_le_bytes(bytes: &[u8]) -> Self;

    fn bytes(_: &Self) -> Vec<u8>;
}

impl FromBytes for i8 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0]])
    }

    fn bytes(x: &i8) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for i16 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1]])
    }

    fn bytes(x: &i16) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for u16 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1]])
    }

    fn bytes(x: &u16) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for u32 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    fn bytes(x: &u32) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for f32 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    fn bytes(x: &f32) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for i32 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    fn bytes(x: &i32) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for f64 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ])
    }

    fn bytes(x: &f64) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

impl FromBytes for u8 {
    fn from_be_bytes(bytes: &[u8]) -> Self {
        Self::from_be_bytes([bytes[0]])
    }
    fn from_le_bytes(bytes: &[u8]) -> Self {
        Self::from_le_bytes([bytes[0]])
    }

    fn bytes(x: &u8) -> Vec<u8> {
        Vec::from(x.to_le_bytes())
    }
}

#[derive(Clone, Debug, PartialEq)]
enum Endianness {
    Big,
    Little,
}

fn load_loadable<T: FromBytes, const SIZE: usize>(
    start_idx: usize,
    data: &[u8],
    endian: Endianness,
) -> Option<T> {
    match endian {
        Endianness::Big => Some(T::from_be_bytes(
            load_bytes::<SIZE>(start_idx, data)?.as_slice(),
        )),
        Endianness::Little => Some(T::from_le_bytes(
            load_bytes::<SIZE>(start_idx, data)?.as_slice(),
        )),
    }
}

fn load_bytes<const S: usize>(start_idx: usize, data: &[u8]) -> Option<[u8; S]> {
    if start_idx + S > data.len() {
        return None;
    }
    let mut buff: [u8; S] = [u8::zero(); S];
    for (byte, buff_val) in buff.iter_mut().enumerate().take(S) {
        let raw_byte = data.get(start_idx + byte);
        match raw_byte {
            None => {
                return None;
            }
            Some(some_byte) => {
                *buff_val = *some_byte;
            }
        }
    }

    Some(buff)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_bytes() {
        i8::bytes(&1);
        u8::bytes(&1);
        i16::bytes(&1);
        u16::bytes(&1);
        i32::bytes(&1);
        u32::bytes(&1);
        f32::bytes(&1.0);
        f64::bytes(&1.0);
    }
}
