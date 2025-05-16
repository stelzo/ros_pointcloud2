//! Types used to represent ROS messages and convert between different ROS crates.
//!
//! This intermediate layer allows various ROS libraries to be integrated to the conversion process.
//!
//! There are 2 functions needed to be implemented for a new ROS message library:
//! - `From` for converting from the library-generated message to [`PointCloud2Msg`](crate::PointCloud2Msg).
//! ```
//! #[cfg(feature = "fancy_ros_msg")]
//! impl From<fancy_ros::sensor_msgs::msg::PointCloud2> for crate::PointCloud2Msg {
//!     fn from(msg: r2r::sensor_msgs::msg::PointCloud2) -> Self {
//!         // Conversion code - avoid any point buffer copy!
//!     }
//! }
//! ```
//!
//! - `From` for converting from the [`PointCloud2Msg`](crate::PointCloud2Msg) to the library-generated message type.
//! ```
//! #[cfg(feature = "fancy_ros_msg")]
//! impl From<crate::PointCloud2Msg> for fancy_ros::sensor_msgs::msg::PointCloud2 {
//!     fn from(msg: crate::PointCloud2Msg) -> Self {
//!         // Conversion code - avoid any point buffer copy!
//!     }
//! }
//! ```

use alloc::string::String;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// [Time](https://docs.ros2.org/latest/api/builtin_interfaces/msg/Time.html) representation for ROS messages.
#[derive(Clone, Debug, Default, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TimeMsg {
    pub sec: i32,
    pub nanosec: u32,
}

#[cfg(feature = "ros2-interfaces-jazzy")]
impl From<ros2_interfaces_jazzy::builtin_interfaces::msg::Time> for TimeMsg {
    fn from(time: ros2_interfaces_jazzy::builtin_interfaces::msg::Time) -> Self {
        Self {
            sec: time.sec,
            nanosec: time.nanosec,
        }
    }
}

#[cfg(feature = "ros2-interfaces-jazzy-rkyv")]
impl From<ros2_interfaces_jazzy_rkyv::builtin_interfaces::msg::Time> for TimeMsg {
    fn from(time: ros2_interfaces_jazzy_rkyv::builtin_interfaces::msg::Time) -> Self {
        Self {
            sec: time.sec,
            nanosec: time.nanosec,
        }
    }
}

#[cfg(feature = "rosrust_msg")]
impl From<rosrust::Time> for TimeMsg {
    fn from(time: rosrust::Time) -> Self {
        Self {
            sec: time.sec as i32,
            nanosec: time.nsec,
        }
    }
}

/// Represents the [header of a ROS message](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html).
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct HeaderMsg {
    pub seq: u32,
    pub stamp: TimeMsg,
    pub frame_id: String,
}

#[cfg(feature = "ros2-interfaces-jazzy")]
impl From<ros2_interfaces_jazzy::std_msgs::msg::Header> for HeaderMsg {
    fn from(header: ros2_interfaces_jazzy::std_msgs::msg::Header) -> Self {
        Self {
            seq: 0,
            stamp: header.stamp.into(),
            frame_id: header.frame_id,
        }
    }
}
#[cfg(feature = "ros2-interfaces-jazzy-rkyv")]
impl From<ros2_interfaces_jazzy_rkyv::std_msgs::msg::Header> for HeaderMsg {
    fn from(header: ros2_interfaces_jazzy_rkyv::std_msgs::msg::Header) -> Self {
        Self {
            seq: 0,
            stamp: header.stamp.into(),
            frame_id: header.frame_id,
        }
    }
}

/// Describing a point encoded in the byte buffer of a PointCloud2 message. See the [official message description](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointField.html) for more information.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PointFieldMsg {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

impl Default for PointFieldMsg {
    fn default() -> Self {
        Self {
            name: String::new(),
            offset: 0,
            datatype: 0,
            count: 1,
        }
    }
}

#[cfg(feature = "ros2-interfaces-jazzy")]
impl From<ros2_interfaces_jazzy::sensor_msgs::msg::PointCloud2> for crate::PointCloud2Msg {
    fn from(msg: ros2_interfaces_jazzy::sensor_msgs::msg::PointCloud2) -> Self {
        Self {
            header: HeaderMsg {
                seq: 0,
                stamp: TimeMsg {
                    sec: msg.header.stamp.sec,
                    nanosec: msg.header.stamp.nanosec,
                },
                frame_id: msg.header.frame_id,
            },
            dimensions: crate::CloudDimensions {
                width: msg.width,
                height: msg.height,
            },
            fields: msg
                .fields
                .into_iter()
                .map(|field| PointFieldMsg {
                    name: field.name,
                    offset: field.offset,
                    datatype: field.datatype,
                    count: field.count,
                })
                .collect(),
            endian: if msg.is_bigendian {
                crate::Endian::Big
            } else {
                crate::Endian::Little
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            dense: if msg.is_dense {
                crate::Denseness::Dense
            } else {
                crate::Denseness::Sparse
            },
        }
    }
}

#[cfg(feature = "ros2-interfaces-jazzy-rkyv")]
impl From<ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointCloud2> for crate::PointCloud2Msg {
    fn from(msg: ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointCloud2) -> Self {
        Self {
            header: HeaderMsg {
                seq: 0,
                stamp: TimeMsg {
                    sec: msg.header.stamp.sec,
                    nanosec: msg.header.stamp.nanosec,
                },
                frame_id: msg.header.frame_id,
            },
            dimensions: crate::CloudDimensions {
                width: msg.width,
                height: msg.height,
            },
            fields: msg
                .fields
                .into_iter()
                .map(|field| PointFieldMsg {
                    name: field.name,
                    offset: field.offset,
                    datatype: field.datatype,
                    count: field.count,
                })
                .collect(),
            endian: if msg.is_bigendian {
                crate::Endian::Big
            } else {
                crate::Endian::Little
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            dense: if msg.is_dense {
                crate::Denseness::Dense
            } else {
                crate::Denseness::Sparse
            },
        }
    }
}

#[cfg(feature = "ros2-interfaces-jazzy")]
impl From<crate::PointCloud2Msg> for ros2_interfaces_jazzy::sensor_msgs::msg::PointCloud2 {
    fn from(msg: crate::PointCloud2Msg) -> Self {
        ros2_interfaces_jazzy::sensor_msgs::msg::PointCloud2 {
            header: ros2_interfaces_jazzy::std_msgs::msg::Header {
                stamp: ros2_interfaces_jazzy::builtin_interfaces::msg::Time {
                    sec: msg.header.stamp.sec,
                    nanosec: msg.header.stamp.nanosec,
                },
                frame_id: msg.header.frame_id,
            },
            height: msg.dimensions.height,
            width: msg.dimensions.width,
            fields: msg
                .fields
                .into_iter()
                .map(
                    |field| ros2_interfaces_jazzy::sensor_msgs::msg::PointField {
                        name: field.name,
                        offset: field.offset,
                        datatype: field.datatype,
                        count: field.count,
                    },
                )
                .collect(),
            is_bigendian: match msg.endian {
                crate::Endian::Big => true,
                crate::Endian::Little => false,
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: match msg.dense {
                crate::Denseness::Dense => true,
                crate::Denseness::Sparse => false,
            },
        }
    }
}

#[cfg(feature = "ros2-interfaces-jazzy-rkyv")]
impl From<crate::PointCloud2Msg> for ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointCloud2 {
    fn from(msg: crate::PointCloud2Msg) -> Self {
        ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointCloud2 {
            header: ros2_interfaces_jazzy_rkyv::std_msgs::msg::Header {
                stamp: ros2_interfaces_jazzy_rkyv::builtin_interfaces::msg::Time {
                    sec: msg.header.stamp.sec,
                    nanosec: msg.header.stamp.nanosec,
                },
                frame_id: msg.header.frame_id,
            },
            height: msg.dimensions.height,
            width: msg.dimensions.width,
            fields: msg
                .fields
                .into_iter()
                .map(
                    |field| ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointField {
                        name: field.name,
                        offset: field.offset,
                        datatype: field.datatype,
                        count: field.count,
                    },
                )
                .collect(),
            is_bigendian: match msg.endian {
                crate::Endian::Big => true,
                crate::Endian::Little => false,
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: match msg.dense {
                crate::Denseness::Dense => true,
                crate::Denseness::Sparse => false,
            },
        }
    }
}

#[cfg(feature = "r2r_msg")]
impl From<r2r::sensor_msgs::msg::PointCloud2> for crate::PointCloud2Msg {
    fn from(msg: r2r::sensor_msgs::msg::PointCloud2) -> Self {
        Self {
            header: HeaderMsg {
                seq: 0,
                stamp: TimeMsg {
                    sec: msg.header.stamp.sec,
                    nanosec: msg.header.stamp.nanosec,
                },
                frame_id: msg.header.frame_id,
            },
            dimensions: crate::CloudDimensions {
                width: msg.width,
                height: msg.height,
            },
            fields: msg
                .fields
                .into_iter()
                .map(|field| PointFieldMsg {
                    name: field.name,
                    offset: field.offset,
                    datatype: field.datatype,
                    count: field.count,
                })
                .collect(),
            endian: if msg.is_bigendian {
                crate::Endian::Big
            } else {
                crate::Endian::Little
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            dense: if msg.is_dense {
                crate::Denseness::Dense
            } else {
                crate::Denseness::Sparse
            },
        }
    }
}

#[cfg(feature = "r2r_msg")]
impl From<crate::PointCloud2Msg> for r2r::sensor_msgs::msg::PointCloud2 {
    fn from(msg: crate::PointCloud2Msg) -> Self {
        r2r::sensor_msgs::msg::PointCloud2 {
            header: r2r::std_msgs::msg::Header {
                stamp: r2r::builtin_interfaces::msg::Time {
                    sec: msg.header.stamp.sec,
                    nanosec: msg.header.stamp.nanosec,
                },
                frame_id: msg.header.frame_id,
            },
            height: msg.dimensions.height,
            width: msg.dimensions.width,
            fields: msg
                .fields
                .into_iter()
                .map(|field| r2r::sensor_msgs::msg::PointField {
                    name: field.name,
                    offset: field.offset,
                    datatype: field.datatype,
                    count: field.count,
                })
                .collect(),
            is_bigendian: match msg.endian {
                crate::Endian::Big => true,
                crate::Endian::Little => false,
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: match msg.dense {
                crate::Denseness::Dense => true,
                crate::Denseness::Sparse => false,
            },
        }
    }
}

#[cfg(feature = "rosrust_msg")]
impl From<rosrust_msg::sensor_msgs::PointCloud2> for crate::PointCloud2Msg {
    fn from(msg: rosrust_msg::sensor_msgs::PointCloud2) -> Self {
        Self {
            header: HeaderMsg {
                seq: msg.header.seq,
                stamp: TimeMsg {
                    sec: msg.header.stamp.sec as i32,
                    nanosec: msg.header.stamp.nsec,
                },
                frame_id: msg.header.frame_id,
            },
            dimensions: crate::CloudDimensions {
                width: msg.width,
                height: msg.height,
            },
            fields: msg
                .fields
                .into_iter()
                .map(|field| PointFieldMsg {
                    name: field.name,
                    offset: field.offset,
                    datatype: field.datatype,
                    count: field.count,
                })
                .collect(),
            endian: if msg.is_bigendian {
                crate::Endian::Big
            } else {
                crate::Endian::Little
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            dense: if msg.is_dense {
                crate::Denseness::Dense
            } else {
                crate::Denseness::Sparse
            },
        }
    }
}

#[cfg(feature = "rosrust_msg")]
impl From<crate::PointCloud2Msg> for rosrust_msg::sensor_msgs::PointCloud2 {
    fn from(msg: crate::PointCloud2Msg) -> Self {
        rosrust_msg::sensor_msgs::PointCloud2 {
            header: rosrust_msg::std_msgs::Header {
                seq: msg.header.seq,
                stamp: rosrust::Time {
                    sec: msg.header.stamp.sec as u32,
                    nsec: msg.header.stamp.nanosec,
                },
                frame_id: msg.header.frame_id,
            },
            height: msg.dimensions.height,
            width: msg.dimensions.width,
            fields: msg
                .fields
                .into_iter()
                .map(|field| rosrust_msg::sensor_msgs::PointField {
                    name: field.name,
                    offset: field.offset,
                    datatype: field.datatype,
                    count: field.count,
                })
                .collect(),
            is_bigendian: if msg.endian == crate::Endian::Big {
                true
            } else {
                false
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: if msg.dense == crate::Denseness::Dense {
                true
            } else {
                false
            },
        }
    }
}
