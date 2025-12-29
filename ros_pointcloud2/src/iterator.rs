//! Iterator implementations for [`PointCloud2Msg`] including a parallel iterator for rayon.
use crate::{
    ConversionError, Endian, FieldDatatype, IPoint, PointCloud2Msg, PointConvertible, PointData,
};

use alloc::string::{String, ToString};
use alloc::vec::Vec;

/// Zero-copy iterator abstraction over a [`PointCloud2Msg`].
pub struct PointCloudIterator<'a, const N: usize, C>
where
    C: PointConvertible<N> + 'a,
{
    iteration: usize,
    iteration_back: usize,
    data: ByteBufferView<'a, N>,
    _phantom: core::marker::PhantomData<C>,
}

struct ByteBufferView<'a, const N: usize> {
    data: &'a [u8],
    start_point_idx: usize,
    end_point_idx: usize,
    point_step_size: usize,
    offsets: [usize; N],
    datatypes: [FieldDatatype; N],
    endian: Endian,
}

impl<'a, const N: usize> ByteBufferView<'a, N> {
    fn new(
        data: &'a [u8],
        point_step_size: usize,
        start_point_idx: usize,
        end_point_idx: usize,
        offsets: [usize; N],
        datatypes: [FieldDatatype; N],
        endian: Endian,
    ) -> Self {
        Self {
            data,
            start_point_idx,
            end_point_idx,
            point_step_size,
            offsets,
            datatypes,
            endian,
        }
    }

    #[inline]
    fn len(&self) -> usize {
        self.end_point_idx - self.start_point_idx + 1
    }

    #[inline]
    fn point_at(&self, idx: usize) -> IPoint<N> {
        let offset = (self.start_point_idx + idx) * self.point_step_size;
        let mut pdata = [PointData::default(); N];
        pdata
            .iter_mut()
            .zip(self.offsets.iter())
            .zip(self.datatypes.iter())
            .for_each(|((pdata_entry, in_point_offset), pdata_type)| {
                *pdata_entry = PointData::from_buffer(
                    self.data,
                    offset + in_point_offset,
                    *pdata_type,
                    self.endian,
                );
            });

        pdata.into()
    }

    #[inline]
    fn clone_with_bounds(&self, start: usize, size: usize) -> Self {
        Self {
            data: self.data,
            start_point_idx: start,
            end_point_idx: start + size - 1,
            point_step_size: self.point_step_size,
            offsets: self.offsets,
            datatypes: self.datatypes,
            endian: self.endian,
        }
    }

    #[inline]
    pub fn split_at(self, point_index: usize) -> (Self, Self) {
        let left_start = self.start_point_idx;
        let left_size = point_index;

        let right_start = self.start_point_idx + point_index;
        let right_size = self.len() - point_index;
        (
            self.clone_with_bounds(left_start, left_size),
            self.clone_with_bounds(right_start, right_size),
        )
    }
}

impl<'a, const N: usize, C> TryFrom<&'a PointCloud2Msg> for PointCloudIterator<'a, N, C>
where
    C: PointConvertible<N> + 'a,
{
    type Error = ConversionError;

    fn try_from(cloud: &'a PointCloud2Msg) -> Result<Self, Self::Error> {
        let layout = C::layout();
        let fields_only = crate::ordered_field_names_from_layout(&layout);

        let mut offsets = [usize::default(); N];
        let mut datatypes = [FieldDatatype::default(); N];
        let mut idx: usize = 0;
        let mut missing: Vec<String> = Vec::new();

        for &name in fields_only.iter() {
            match cloud.fields.iter().find(|f| f.name == name) {
                Some(field) => {
                    datatypes[idx] = field.datatype.try_into()?;
                    offsets[idx] = field.offset as usize;
                    idx += 1;
                }
                None => missing.push(name.to_string()),
            }
        }

        if !missing.is_empty() {
            return Err(ConversionError::FieldsNotFound(missing));
        }

        let point_step_size = cloud.point_step as usize;
        if point_step_size * cloud.dimensions.len() != cloud.data.len() {
            return Err(ConversionError::DataLengthMismatch);
        }

        // Ensure that the last byte used by any field fits into the point step.
        let max_end = datatypes
            .iter()
            .zip(offsets.iter())
            .map(|(dt, off)| off + dt.size())
            .max()
            .unwrap_or(0);
        if max_end > point_step_size {
            return Err(ConversionError::DataLengthMismatch);
        }

        let data = ByteBufferView::new(
            cloud.data.as_slice(),
            point_step_size,
            0,
            cloud.dimensions.len() - 1,
            offsets,
            datatypes,
            cloud.endian,
        );

        Ok(Self {
            iteration: 0,
            iteration_back: cloud.dimensions.len() - 1,
            data,
            _phantom: core::marker::PhantomData,
        })
    }
}

impl<'a, const N: usize, C> Iterator for PointCloudIterator<'a, N, C>
where
    C: PointConvertible<N> + 'a,
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

impl<'a, const N: usize, C> PointCloudIterator<'a, N, C>
where
    C: PointConvertible<N> + 'a,
{
    #[inline]
    #[must_use]
    fn from_byte_buffer_view(data: ByteBufferView<'a, N>) -> Self {
        Self {
            iteration: 0,
            iteration_back: data.len() - 1,
            data,
            _phantom: core::marker::PhantomData,
        }
    }

    #[inline]
    #[must_use]
    pub fn split_at(self, point_index: usize) -> (Self, Self) {
        let (left_data, right_data) = self.data.split_at(point_index);
        (
            Self::from_byte_buffer_view(left_data),
            Self::from_byte_buffer_view(right_data),
        )
    }
}

#[cfg(feature = "rayon")]
impl<'a, const N: usize, C> ExactSizeIterator for PointCloudIterator<'a, N, C>
where
    C: PointConvertible<N> + Send + Sync + 'a,
{
    fn len(&self) -> usize {
        self.data.len()
    }
}

#[cfg(feature = "rayon")]
impl<'a, const N: usize, C> DoubleEndedIterator for PointCloudIterator<'a, N, C>
where
    C: PointConvertible<N> + Send + Sync + 'a,
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
impl<'a, const N: usize, C> rayon::iter::ParallelIterator for PointCloudIterator<'a, N, C>
where
    C: PointConvertible<N> + Send + Sync + 'a,
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
impl<'a, const N: usize, C> rayon::iter::IndexedParallelIterator for PointCloudIterator<'a, N, C>
where
    C: PointConvertible<N> + Send + Sync + 'a,
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
struct RayonProducer<'a, const N: usize, C: PointConvertible<N> + Send + Sync + 'a> {
    iter: PointCloudIterator<'a, N, C>,
}

#[cfg(feature = "rayon")]
impl<'a, const N: usize, C> rayon::iter::plumbing::Producer for RayonProducer<'a, N, C>
where
    C: PointConvertible<N> + Send + Sync + 'a,
{
    type Item = C;
    type IntoIter = PointCloudIterator<'a, N, C>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter
    }

    fn split_at(self, point_index: usize) -> (Self, Self) {
        let (left, right) = self.iter.split_at(point_index);
        (RayonProducer { iter: left }, RayonProducer { iter: right })
    }
}

#[cfg(feature = "rayon")]
impl<'a, const N: usize, C> From<PointCloudIterator<'a, N, C>> for RayonProducer<'a, N, C>
where
    C: PointConvertible<N> + Send + Sync + 'a,
{
    fn from(iterator: PointCloudIterator<'a, N, C>) -> Self {
        Self { iter: iterator }
    }
}

#[cfg(feature = "rayon")]
mod test {
    #![allow(clippy::unwrap_used)]

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

        let internal_msg = crate::PointCloud2Msg::try_from_iter(&cloud).unwrap();
        let mut iter = crate::iterator::PointCloudIterator::try_from(&internal_msg).unwrap();
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

        let internal_msg = crate::PointCloud2Msg::try_from_iter(&cloud).unwrap();
        let iter = crate::iterator::PointCloudIterator::try_from(&internal_msg).unwrap();

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
