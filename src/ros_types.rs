#[cfg(not(feature = "rosrust_msg"))]
#[derive(Clone, Debug, Default)]
pub struct TimeMsg {
    pub sec: u32,
    pub nsec: u32,
}

#[cfg(feature = "rosrust_msg")]
pub use rosrust::Time as TimeMsg;

#[derive(Clone, Debug, Default)]
pub struct HeaderMsg {
    pub seq: u32,
    pub stamp: TimeMsg,
    pub frame_id: String,
}

#[derive(Clone, Debug)]
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

#[cfg(feature = "r2r_msg")]
impl From<r2r::sensor_msgs::msg::PointCloud2> for crate::PointCloud2Msg {
    fn from(msg: r2r::sensor_msgs::msg::PointCloud2) -> Self {
        Self {
            header: HeaderMsg {
                seq: 0,
                stamp: TimeMsg {
                    sec: msg.header.stamp.sec as u32,
                    nsec: msg.header.stamp.nanosec,
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
                crate::convert::Denseness::Dense
            } else {
                crate::convert::Denseness::Sparse
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
                    sec: msg.header.stamp.sec as i32,
                    nanosec: msg.header.stamp.nsec,
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
            is_bigendian: if msg.endian == crate::Endian::Big {
                true
            } else {
                false
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: if msg.dense == crate::convert::Denseness::Dense {
                true
            } else {
                false
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
                    sec: msg.header.stamp.sec,
                    nsec: msg.header.stamp.nsec,
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
            is_bigendian: if msg.endian == crate::Endian::Big {
                true
            } else {
                false
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: if msg.dense == crate::convert::Denseness::Dense {
                true
            } else {
                false
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
                stamp: TimeMsg {
                    sec: msg.header.stamp.sec,
                    nsec: msg.header.stamp.nsec,
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
            is_dense: if msg.dense == crate::convert::Denseness::Dense {
                true
            } else {
                false
            },
        }
    }
}
