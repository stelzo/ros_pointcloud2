use crate::{
    convert::{check_coord, load_loadable, Endianness, FieldDatatype, FromBytes},
    ConversionError, MetaNames, Point, PointCloud2Msg, PointConvertible, PointMeta,
};

/// The PointCloudIterator provides a an iterator abstraction of the PointCloud2Msg.
///
/// The iterator is defined at compile time, so the point has to be described via template arguments.
///
/// When using within a ROS node, the PointCloud2 (created by the ROS crate) must be converted first.
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
/// ros_pointcloud2 supports r2r, rclrs and rosrust as conversion targets out of the box via feature flags.
///
pub struct PointCloudIterator<
    T: FromBytes,
    const SIZE: usize,
    const DIM: usize,
    const METADIM: usize,
    C,
> where
    T: FromBytes,
    C: MetaNames<METADIM>,
{
    iteration: usize,
    iteration_back: usize,
    data: Vec<u8>,
    point_step_size: usize,
    cloud_length: usize,
    offsets: Vec<usize>,
    meta: Vec<(String, FieldDatatype)>,
    endianness: Endianness,
    phantom_t: std::marker::PhantomData<T>, // internally used for byte and datatype conversions
    phantom_c: std::marker::PhantomData<C>, // internally used for meta names array
}

#[cfg(feature = "rayon")]
impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> ExactSizeIterator
    for PointCloudIterator<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes + Send + Sync,
    C: PointConvertible<T, SIZE, DIM, METADIM> + Send + Sync,
{
    fn len(&self) -> usize {
        self.cloud_length
    }
}

#[cfg(feature = "rayon")]
impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> DoubleEndedIterator
    for PointCloudIterator<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes + Send + Sync,
    C: PointConvertible<T, SIZE, DIM, METADIM> + Send + Sync,
{
    fn next_back(&mut self) -> Option<Self::Item> {
        if self.iteration_back < self.iteration {
            return None; // iteration finished
        }

        let p = self.point_at(self.iteration_back);
        Some(C::from(p))
    }
}

#[cfg(feature = "rayon")]
impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> rayon::iter::ParallelIterator
    for PointCloudIterator<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes + Send + Sync,
    C: PointConvertible<T, SIZE, DIM, METADIM> + Send + Sync,
{
    type Item = C;

    fn drive_unindexed<Co>(self, consumer: Co) -> Co::Result
    where
        Co: rayon::iter::plumbing::UnindexedConsumer<Self::Item>,
    {
        rayon::iter::plumbing::bridge(self, consumer)
    }

    fn opt_len(&self) -> Option<usize> {
        Some(self.cloud_length)
    }
}

#[cfg(feature = "rayon")]
impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C>
    rayon::iter::IndexedParallelIterator for PointCloudIterator<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes + Send + Sync,
    C: PointConvertible<T, SIZE, DIM, METADIM> + Send + Sync,
{
    fn len(&self) -> usize {
        self.cloud_length
    }

    fn drive<Co>(self, consumer: Co) -> Co::Result
    where
        Co: rayon::iter::plumbing::Consumer<Self::Item>,
    {
        rayon::iter::plumbing::bridge(self, consumer)
    }

    fn with_producer<CB: rayon::iter::plumbing::ProducerCallback<Self::Item>>(
        self,
        callback: CB,
    ) -> CB::Output {
        callback.callback(RayonProducer::from(self))
    }
}

#[cfg(feature = "rayon")]
struct RayonProducer<
    T: FromBytes,
    const SIZE: usize,
    const DIM: usize,
    const METADIM: usize,
    C: PointConvertible<T, SIZE, DIM, METADIM>,
> {
    iter: PointCloudIterator<T, SIZE, DIM, METADIM, C>,
}

#[cfg(feature = "rayon")]
impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C>
    rayon::iter::plumbing::Producer for RayonProducer<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes + Send + Sync,
    C: PointConvertible<T, SIZE, DIM, METADIM> + Send + Sync,
{
    type Item = C;
    type IntoIter = PointCloudIterator<T, SIZE, DIM, METADIM, C>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter
    }

    fn split_at(self, index: usize) -> (Self, Self) {
        let (left, right) = self.iter.split_at(index);
        (RayonProducer { iter: left }, RayonProducer { iter: right })
    }
}

#[cfg(feature = "rayon")]
impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C>
    From<PointCloudIterator<T, SIZE, DIM, METADIM, C>> for RayonProducer<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes + Send + Sync,
    C: PointConvertible<T, SIZE, DIM, METADIM> + Send + Sync,
{
    fn from(iterator: PointCloudIterator<T, SIZE, DIM, METADIM, C>) -> Self {
        Self { iter: iterator }
    }
}

/// Implementation of the iterator trait.
impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> Iterator
    for PointCloudIterator<T, SIZE, DIM, METADIM, C>
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

        let p = self.point_at(self.iteration);

        self.iteration += 1;

        Some(C::from(p))
    }
}

impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C> TryFrom<PointCloud2Msg>
    for PointCloudIterator<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: MetaNames<METADIM>,
{
    type Error = ConversionError;

    /// Convert a PointCloud2Msg into an iterator.
    /// Converting a PointCloud2Msg into an iterator is a fallible operation since the message can contain only a subset of the required fields.
    ///
    /// The theoretical time complexity is O(n) where n is the number of fields defined in the message for a single point which is typically small.
    /// It therefore has a constant time complexity O(1) for practical purposes.
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
                        meta_with_offsets[meta_idx].0.clone_from(&field.name);
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

        let cloud_length = cloud.width as usize * cloud.height as usize;

        Ok(Self {
            iteration: 0,
            iteration_back: cloud_length - 1,
            data: cloud.data,
            point_step_size,
            cloud_length,
            offsets,
            meta,
            endianness: endian,
            phantom_t: std::marker::PhantomData,
            phantom_c: std::marker::PhantomData,
        })
    }
}

impl<T, const SIZE: usize, const DIM: usize, const METADIM: usize, C>
    PointCloudIterator<T, SIZE, DIM, METADIM, C>
where
    T: FromBytes,
    C: MetaNames<METADIM>,
{
    #[inline(always)]
    fn point_at(&self, offset: usize) -> Point<T, DIM, METADIM> {
        let mut xyz = [T::default(); DIM];
        xyz.iter_mut()
            .zip(self.offsets.iter())
            .for_each(|(p_xyz, in_point_offset)| {
                *p_xyz = load_loadable::<T, SIZE>(
                    (offset * self.point_step_size) + in_point_offset,
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
                let start = (offset * self.point_step_size) + in_point_offset;
                *p_meta = PointMeta::from_buffer(&self.data, start, meta_type);
            });

        Point { coords: xyz, meta }
    }

    pub fn split_at(self, index: usize) -> (Self, Self) {
        let data_idx = index * self.point_step_size;
        let (left, right) = self.data.split_at(data_idx);
        let left_iteration = self.iteration;
        let right_iteration = self.iteration + index;
        let right_iteration_back = self.iteration_back;
        (
            Self {
                iteration: left_iteration,
                iteration_back: right_iteration,
                data: left.into(), // TODO richtig mist hier
                point_step_size: self.point_step_size.clone(),
                cloud_length: right_iteration - left_iteration,
                offsets: self.offsets.clone(),
                meta: self.meta.clone(),
                endianness: self.endianness.clone(),
                phantom_t: std::marker::PhantomData,
                phantom_c: std::marker::PhantomData,
            },
            Self {
                iteration: right_iteration,
                iteration_back: right_iteration_back,
                data: right.into(), // TODO richtig mist hier
                point_step_size: self.point_step_size,
                cloud_length: right_iteration_back - right_iteration,
                offsets: self.offsets.clone(),
                meta: self.meta.clone(),
                endianness: self.endianness.clone(),
                phantom_t: std::marker::PhantomData,
                phantom_c: std::marker::PhantomData,
            },
        )
    }

    /// Split the iterator at the given index.#
    pub fn from_subslice(
        data: Vec<u8>,
        start: usize,
        end: usize,
        offsets: Vec<usize>,
        meta: Vec<(String, FieldDatatype)>,
        endianness: Endianness,
    ) -> Self {
        Self {
            iteration: start,
            iteration_back: end,
            data: data,
            point_step_size: 0,
            cloud_length: end - start,
            offsets: offsets,
            meta: meta,
            endianness: endianness,
            phantom_t: std::marker::PhantomData,
            phantom_c: std::marker::PhantomData,
        }
    }
}
