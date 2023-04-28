#[derive(Clone, Debug)]
pub struct TimeMsg {
    pub sec: u32,
    pub nsec: u32,
}

impl Default for TimeMsg {
    fn default() -> Self {
        Self { sec: 0, nsec: 0 }
    }
}

#[derive(Clone, Debug)]
pub struct HeaderMsg {
    pub seq: u32,
    pub stamp: TimeMsg,
    pub frame_id: String,
}

impl Default for HeaderMsg {
    fn default() -> Self {
        Self {
            seq: 0,
            stamp: TimeMsg::default(),
            frame_id: String::new(),
        }
    }
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
            count: 0,
        }
    }
}

#[derive(Clone, Debug)]
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

impl Default for PointCloud2Msg {
    fn default() -> Self {
        Self {
            header: HeaderMsg::default(),
            height: 0,
            width: 0,
            fields: Vec::new(),
            is_bigendian: false,
            point_step: 0,
            row_step: 0,
            data: Vec::new(),
            is_dense: false,
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
impl Into<rosrust_msg::sensor_msgs::PointCloud2> for PointCloud2Msg {
    fn into(self) -> rosrust_msg::sensor_msgs::PointCloud2 {
        rosrust_msg::sensor_msgs::PointCloud2 {
            header: rosrust_msg::std_msgs::Header {
                seq: self.header.seq,
                stamp: rosrust::Time {
                    sec: self.header.stamp.sec,
                    nsec: self.header.stamp.nsec,
                },
                frame_id: self.header.frame_id,
            },
            height: self.height,
            width: self.width,
            fields: self
                .fields
                .into_iter()
                .map(|field| rosrust_msg::sensor_msgs::PointField {
                    name: field.name,
                    offset: field.offset,
                    datatype: field.datatype,
                    count: field.count,
                })
                .collect(),
            is_bigendian: self.is_bigendian,
            point_step: self.point_step,
            row_step: self.row_step,
            data: self.data,
            is_dense: self.is_dense,
        }
    }
}
