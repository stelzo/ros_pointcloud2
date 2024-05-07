use crate::{
    convert::{Endianness, FieldDatatype},
    Fields, MsgConversionError, PointCloud2Msg, PointConvertible, PointMeta, RPCL2Point,
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
pub struct PointCloudIterator<const N: usize, C>
where
    C: Fields<N>,
{
    iteration: usize,
    iteration_back: usize,
    data: ByteBufferView<N>,
    phantom_c: std::marker::PhantomData<C>, // internally used for meta names array
}

#[cfg(feature = "rayon")]
impl<const N: usize, C> ExactSizeIterator for PointCloudIterator<N, C>
where
    C: PointConvertible<N> + Send + Sync,
{
    fn len(&self) -> usize {
        self.data.len()
    }
}

#[cfg(feature = "rayon")]
impl<const N: usize, C> DoubleEndedIterator for PointCloudIterator<N, C>
where
    C: PointConvertible<N> + Send + Sync,
{
    fn next_back(&mut self) -> Option<Self::Item> {
        if self.iteration_back < self.iteration {
            return None; // iteration finished
        }

        let p = self.data.point_at(self.iteration_back);
        self.iteration_back -= 1;
        Some(C::from(p))
    }
}

#[cfg(feature = "rayon")]
impl<const N: usize, C> rayon::iter::ParallelIterator for PointCloudIterator<N, C>
where
    C: PointConvertible<N> + Send + Sync,
{
    type Item = C;

    fn drive_unindexed<Co>(self, consumer: Co) -> Co::Result
    where
        Co: rayon::iter::plumbing::UnindexedConsumer<Self::Item>,
    {
        rayon::iter::plumbing::bridge(self, consumer)
    }

    fn opt_len(&self) -> Option<usize> {
        Some(self.data.len())
    }
}

#[cfg(feature = "rayon")]
impl<const N: usize, C> rayon::iter::IndexedParallelIterator for PointCloudIterator<N, C>
where
    C: PointConvertible<N> + Send + Sync,
{
    fn len(&self) -> usize {
        self.data.len()
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
struct RayonProducer<const N: usize, C: PointConvertible<N>> {
    iter: PointCloudIterator<N, C>,
}

#[cfg(feature = "rayon")]
impl<const N: usize, C> rayon::iter::plumbing::Producer for RayonProducer<N, C>
where
    C: PointConvertible<N> + Send + Sync,
{
    type Item = C;
    type IntoIter = PointCloudIterator<N, C>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter
    }

    fn split_at(self, point_index: usize) -> (Self, Self) {
        let (left, right) = self.iter.split_at(point_index);
        (RayonProducer { iter: left }, RayonProducer { iter: right })
    }
}

#[cfg(feature = "rayon")]
impl<const N: usize, C> From<PointCloudIterator<N, C>> for RayonProducer<N, C>
where
    C: PointConvertible<N> + Send + Sync,
{
    fn from(iterator: PointCloudIterator<N, C>) -> Self {
        Self { iter: iterator }
    }
}

/// Implementation of the iterator trait.
impl<const N: usize, C> Iterator for PointCloudIterator<N, C>
where
    C: PointConvertible<N>,
{
    type Item = C;

    fn size_hint(&self) -> (usize, Option<usize>) {
        let buf_len = self.data.len();
        (buf_len, Some(buf_len))
    }

    fn next(&mut self) -> Option<Self::Item> {
        if self.iteration >= self.data.len() || self.iteration_back < self.iteration {
            return None; // iteration finished
        }

        let p = self.data.point_at(self.iteration);
        self.iteration += 1;
        Some(C::from(p))
    }
}

struct ByteBufferView<const N: usize> {
    data: std::sync::Arc<[u8]>,
    start_point_idx: usize,
    end_point_idx: usize,
    point_step_size: usize,
    offsets: [usize; N],
    meta: Vec<(String, FieldDatatype)>,
    endianness: Endianness,
}

impl<const N: usize> ByteBufferView<N> {
    fn new(
        data: Vec<u8>,
        point_step_size: usize,
        start_point_idx: usize,
        end_point_idx: usize,
        offsets: [usize; N],
        meta: Vec<(String, FieldDatatype)>,
        endianness: Endianness,
    ) -> Self {
        Self {
            data: std::sync::Arc::<[u8]>::from(data),
            start_point_idx,
            end_point_idx,
            point_step_size,
            offsets,
            meta,
            endianness,
        }
    }

    #[inline]
    fn len(&self) -> usize {
        self.end_point_idx - self.start_point_idx + 1
    }

    #[inline(always)]
    fn point_at(&self, idx: usize) -> RPCL2Point<N> {
        let offset = (self.start_point_idx + idx) * self.point_step_size;

        // TODO memcpy entire point at once, then extract fields?
        let mut meta = [PointMeta::default(); N];
        meta.iter_mut()
            .zip(self.offsets.iter())
            .zip(self.meta.iter())
            .for_each(|((p_meta, in_point_offset), (_, meta_type))| {
                *p_meta = PointMeta::from_buffer(
                    &self.data,
                    offset + in_point_offset,
                    *meta_type,
                    self.endianness,
                );
            });

        RPCL2Point { fields: meta }
    }

    #[inline]
    fn clone_with_bounds(&self, start: usize, size: usize) -> Self {
        Self {
            data: self.data.clone(),
            start_point_idx: start,
            end_point_idx: start + size - 1,
            point_step_size: self.point_step_size,
            offsets: self.offsets,
            meta: self.meta.clone(),
            endianness: self.endianness,
        }
    }

    #[inline]
    pub fn split_at(self, point_index: usize) -> (Self, Self) {
        let left_start = self.start_point_idx;
        let left_size = point_index;

        let right_start = point_index;
        let right_size = self.len() - point_index;

        (
            self.clone_with_bounds(left_start, left_size),
            self.clone_with_bounds(right_start, right_size),
        )
    }
}

impl<const N: usize, C> TryFrom<PointCloud2Msg> for PointCloudIterator<N, C>
where
    C: Fields<N>,
{
    type Error = MsgConversionError;

    /// Convert a PointCloud2Msg into an iterator.
    /// Converting a PointCloud2Msg into an iterator is a fallible operation since the message can contain only a subset of the required fields.
    ///
    /// The theoretical time complexity is O(n) where n is the number of fields defined in the message for a single point which is typically small.
    /// It therefore has a constant time complexity O(1) for practical purposes.
    fn try_from(cloud: PointCloud2Msg) -> Result<Self, Self::Error> {
        let mut meta_with_offsets = vec![(String::default(), FieldDatatype::default(), 0); N];

        let not_found_fieldnames = C::field_names_ordered()
            .into_iter()
            .map(|name| {
                let found = cloud.fields.iter().any(|field| field.name == *name);
                (name, found)
            })
            .filter(|(_, found)| !*found)
            .collect::<Vec<_>>();

        if !not_found_fieldnames.is_empty() {
            let names_not_found = not_found_fieldnames
                .into_iter()
                .map(|(name, _)| (*name).to_owned())
                .collect::<Vec<String>>();
            return Err(MsgConversionError::FieldNotFound(names_not_found));
        }

        let ordered_fieldnames = C::field_names_ordered();
        for (field, with_offset) in cloud.fields.iter().zip(meta_with_offsets.iter_mut()) {
            if ordered_fieldnames.contains(&field.name.as_str()) {
                *with_offset = (
                    field.name.clone(),
                    field.datatype.try_into()?,
                    field.offset as usize,
                );
            }
        }

        meta_with_offsets.sort_unstable_by(|(_, _, offset1), (_, _, offset2)| offset1.cmp(offset2));

        debug_assert!(
            meta_with_offsets.len() == N,
            "Not all fields were found in the message. Expected {} but found {}.",
            N,
            meta_with_offsets.len()
        );

        let mut offsets = [usize::default(); N];
        let mut meta = vec![(String::default(), FieldDatatype::default()); N];

        meta_with_offsets
            .into_iter()
            .zip(meta.iter_mut())
            .zip(offsets.iter_mut())
            .for_each(|(((name, datatype, offset), meta), meta_offset)| {
                *meta = (name, datatype);
                *meta_offset = offset;
            });

        let endian = if cloud.is_bigendian {
            Endianness::Big
        } else {
            Endianness::Little
        };

        let point_step_size = cloud.point_step as usize;
        let cloud_length = cloud.width as usize * cloud.height as usize;
        if point_step_size * cloud_length != cloud.data.len() {
            return Err(MsgConversionError::DataLengthMismatch);
        }

        let last_offset = offsets.last().expect("Dimensionality is 0.");

        let last_meta = meta.last().expect("Dimensionality is 0.");
        let size_with_last_meta = last_offset + last_meta.1.size();
        if size_with_last_meta > point_step_size {
            return Err(MsgConversionError::DataLengthMismatch);
        }

        let cloud_length = cloud.width as usize * cloud.height as usize;

        let data = ByteBufferView::new(
            cloud.data,
            point_step_size,
            0,
            cloud_length - 1,
            offsets,
            meta,
            endian,
        );

        Ok(Self {
            iteration: 0,
            iteration_back: cloud_length - 1,
            data,
            phantom_c: std::marker::PhantomData,
        })
    }
}

impl<const N: usize, C> PointCloudIterator<N, C>
where
    C: Fields<N>,
{
    #[inline]
    fn from_byte_buffer_view(data: ByteBufferView<N>) -> Self {
        Self {
            iteration: 0,
            iteration_back: data.len() - 1,
            data,
            phantom_c: std::marker::PhantomData,
        }
    }

    #[inline]
    pub fn split_at(self, point_index: usize) -> (Self, Self) {
        let (left_data, right_data) = self.data.split_at(point_index);
        (
            Self::from_byte_buffer_view(left_data),
            Self::from_byte_buffer_view(right_data),
        )
    }
}

#[cfg(feature = "rayon")]
mod test {

    #[test]
    fn test_double_ended_iter() {
        let cloud = vec![
            crate::points::PointXYZ {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            crate::points::PointXYZ {
                x: 2.0,
                y: 2.0,
                z: 2.0,
            },
            crate::points::PointXYZ {
                x: 3.0,
                y: 3.0,
                z: 3.0,
            },
        ];

        let internal_msg = crate::PointCloud2Msg::try_from_iter(cloud).unwrap();
        let mut iter = crate::iterator::PointCloudIterator::try_from(internal_msg).unwrap();
        let last_p = iter.next_back();

        assert!(last_p.is_some());
        let last_p: crate::points::PointXYZ = last_p.unwrap();

        assert_eq!(last_p.x, 3.0);
        assert_eq!(last_p.y, 3.0);
        assert_eq!(last_p.z, 3.0);

        let first_p = iter.next();
        assert!(first_p.is_some());
        let first_p: crate::points::PointXYZ = first_p.unwrap();

        assert_eq!(first_p.x, 1.0);
        assert_eq!(first_p.y, 1.0);
        assert_eq!(first_p.z, 1.0);

        let last_p = iter.next_back();
        assert!(last_p.is_some());
        let last_p: crate::points::PointXYZ = last_p.unwrap();

        assert_eq!(last_p.x, 2.0);
        assert_eq!(last_p.y, 2.0);
        assert_eq!(last_p.z, 2.0);

        let last_p = iter.next_back();
        assert!(last_p.is_none());

        let first_p = iter.next();
        assert!(first_p.is_none());
    }

    #[test]
    fn test_split_at() {
        let cloud = vec![
            crate::points::PointXYZ {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            crate::points::PointXYZ {
                x: 2.0,
                y: 2.0,
                z: 2.0,
            },
            crate::points::PointXYZ {
                x: 3.0,
                y: 3.0,
                z: 3.0,
            },
        ];

        let internal_msg = crate::PointCloud2Msg::try_from_iter(cloud).unwrap();
        let iter = crate::iterator::PointCloudIterator::try_from(internal_msg).unwrap();

        let (mut left, mut right) = iter.split_at(1);

        assert_eq!(left.size_hint(), (1, Some(1)));
        assert_eq!(right.size_hint(), (2, Some(2)));

        let first_left = left.next();
        assert!(first_left.is_some());
        let first_left: crate::points::PointXYZ = first_left.unwrap();

        assert_eq!(first_left.x, 1.0);
        assert_eq!(first_left.y, 1.0);
        assert_eq!(first_left.z, 1.0);

        let first_right = right.next();
        assert!(first_right.is_some());
        let first_right: crate::points::PointXYZ = first_right.unwrap();

        assert_eq!(first_right.x, 2.0);
        assert_eq!(first_right.y, 2.0);
        assert_eq!(first_right.z, 2.0);

        let last_right = right.next_back();
        assert!(last_right.is_some());

        let last_right: crate::points::PointXYZ = last_right.unwrap();
        assert_eq!(last_right.x, 3.0);
        assert_eq!(last_right.y, 3.0);
        assert_eq!(last_right.z, 3.0);

        let last_left = left.next_back();
        assert!(last_left.is_none());

        let last_right = right.next_back();
        assert!(last_right.is_none());

        let first_left = left.next();
        assert!(first_left.is_none());

        let first_right = right.next();
        assert!(first_right.is_none());
    }
}
