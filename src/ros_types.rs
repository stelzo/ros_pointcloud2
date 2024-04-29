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

#[derive(Clone, Debug, Default)]
pub struct PointFieldMsg {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

#[derive(Clone, Debug, Default)]
pub struct PointCloud2Msg {
    pub header: HeaderMsg,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointFieldMsg>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}

#[cfg(feature = "r2r_msg")]
impl From<r2r::sensor_msgs::msg::PointCloud2> for PointCloud2Msg {
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
            height: msg.height,
            width: msg.width,
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
            is_bigendian: msg.is_bigendian,
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: msg.is_dense,
        }
    }
}

#[cfg(feature = "r2r_msg")]
impl From<PointCloud2Msg> for r2r::sensor_msgs::msg::PointCloud2 {
    fn from(msg: PointCloud2Msg) -> Self {
        r2r::sensor_msgs::msg::PointCloud2 {
            header: r2r::std_msgs::msg::Header {
                stamp: r2r::builtin_interfaces::msg::Time {
                    sec: msg.header.stamp.sec as i32,
                    nanosec: msg.header.stamp.nsec,
                },
                frame_id: msg.header.frame_id,
            },
            height: msg.height,
            width: msg.width,
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
            is_bigendian: msg.is_bigendian,
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: msg.is_dense,
        }
    }
}

#[cfg(feature = "rosrust_msg")]
impl From<rosrust_msg::sensor_msgs::PointCloud2> for PointCloud2Msg {
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
            height: msg.height,
            width: msg.width,
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
            is_bigendian: msg.is_bigendian,
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: msg.is_dense,
        }
    }
}

#[cfg(feature = "rosrust_msg")]
impl From<PointCloud2Msg> for rosrust_msg::sensor_msgs::PointCloud2 {
    fn from(msg: PointCloud2Msg) -> Self {
        rosrust_msg::sensor_msgs::PointCloud2 {
            header: rosrust_msg::std_msgs::Header {
                seq: msg.header.seq,
                stamp: TimeMsg {
                    sec: msg.header.stamp.sec,
                    nsec: msg.header.stamp.nsec,
                },
                frame_id: msg.header.frame_id,
            },
            height: msg.height,
            width: msg.width,
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
            is_bigendian: msg.is_bigendian,
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: msg.is_dense,
        }
    }
}
