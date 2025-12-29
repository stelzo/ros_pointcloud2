//! Types used to represent ROS messages and convert between different ROS crates.
//!
//! This intermediate layer allows various ROS libraries to be integrated to the conversion process.
//!
//!   1. Write your conversion macro in this file to generate conversion functions between `PointCloud2Msg` and the target ROS crate.
//!       - A typical name is `impl_pointcloud2_for_<crate>!()`.
//!   2. Test your implementation in `integration/ros_integration_tests/`. See existing tests for examples and use Docker if needed to set up ROS environments.
//!   3. Create a Workflow in `.github/workflows/` to run the tests with the appropriate features enabled.
//!   4. Create a PR to add the new feature to `Cargo.toml` and document it in `lib.rs`.
//!
use alloc::string::String;

/// [Time](https://docs.ros2.org/latest/api/builtin_interfaces/msg/Time.html) representation for ROS messages.
#[derive(Clone, Debug, Default, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub struct TimeMsg {
    pub sec: i32,
    pub nanosec: u32,
}

/// Represents the [header of a ROS message](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html).
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub struct HeaderMsg {
    pub seq: u32,
    pub stamp: TimeMsg,
    pub frame_id: String,
}

/// Describing a point encoded in the byte buffer of a PointCloud2 message. See the [official message description](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointField.html) for more information.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
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

#[cfg(feature = "safe-drive-msg")]
impl From<safe_drive::msg::common_interfaces::sensor_msgs::msg::PointCloud2>
    for crate::PointCloud2Msg
{
    fn from(msg: safe_drive::msg::common_interfaces::sensor_msgs::msg::PointCloud2) -> Self {
        Self {
            header: HeaderMsg {
                seq: 0,
                stamp: TimeMsg {
                    sec: msg.header.stamp.sec,
                    nanosec: msg.header.stamp.nanosec,
                },
                frame_id: msg.header.frame_id.get_string(),
            },
            dimensions: crate::CloudDimensions {
                width: msg.width,
                height: msg.height,
            },
            fields: msg
                .fields
                .iter()
                .map(|field| PointFieldMsg {
                    name: field.name.get_string(),
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
            data: msg.data.as_slice().to_vec(),
            dense: if msg.is_dense {
                crate::Denseness::Dense
            } else {
                crate::Denseness::Sparse
            },
        }
    }
}

#[cfg(feature = "safe-drive-msg")]
impl From<crate::PointCloud2Msg>
    for safe_drive::msg::common_interfaces::sensor_msgs::msg::PointCloud2
{
    fn from(msg: crate::PointCloud2Msg) -> Self {
        let fields = safe_drive::msg::common_interfaces::sensor_msgs::msg::PointFieldSeq::<0>::new(
            msg.fields.len(),
        );
        let Some(mut fields) = fields else {
            core::panic!("Invalid fields length");
        };
        dbg!(&fields.as_slice_mut()[0]);
        // The memory is not really initialized. The values are all over the place. The String, for example, has size 1 and capacity 0.
        msg.fields
            .into_iter()
            .zip(fields.iter_mut())
            .for_each(|(src_field, tgt_field)| {
                #[cfg(debug_assertions)]
                {
                    let succ = tgt_field.name.assign(&src_field.name);
                    debug_assert!(succ);
                }
                #[cfg(not(debug_assertions))]
                tgt_field.name.assign(&src_field.name);
                tgt_field.offset = src_field.offset;
                tgt_field.datatype = src_field.datatype;
                tgt_field.count = src_field.count;
            });

        let cloud = safe_drive::msg::common_interfaces::sensor_msgs::msg::PointCloud2::new();
        let Some(mut cloud) = cloud else {
            core::panic!("C PointCloud2 creation failed");
        };
        let frame_id = safe_drive::msg::RosString::<0>::new(&msg.header.frame_id);
        let Some(frame_id) = frame_id else {
            core::panic!("C String alloc failed");
        };
        cloud.header = safe_drive::msg::common_interfaces::std_msgs::msg::Header {
            stamp: safe_drive::msg::builtin_interfaces__msg__Time {
                sec: msg.header.stamp.sec,
                nanosec: msg.header.stamp.nanosec,
            },
            frame_id,
        };
        cloud.height = msg.dimensions.height;
        cloud.width = msg.dimensions.width;
        cloud.fields = fields;
        cloud.is_bigendian = match msg.endian {
            crate::Endian::Big => true,
            crate::Endian::Little => false,
        };
        cloud.point_step = msg.point_step;
        cloud.row_step = msg.row_step;

        // NOTE This memcpy can not be avoided with the current safe_drive API because it uses the C allocator
        let data = safe_drive::msg::U8Seq::<0>::new(msg.data.len());
        let Some(mut data) = data else {
            core::panic!("Could not allocate buffer");
        };
        data.as_slice_mut().copy_from_slice(&msg.data);
        cloud.data = data;

        cloud.is_dense = match msg.dense {
            crate::Denseness::Dense => true,
            crate::Denseness::Sparse => false,
        };
        cloud
    }
}

#[macro_export]
macro_rules! impl_pointcloud2_for_r2r {
    () => {
        pub mod impl_r2r {
            pub fn to_pointcloud2_msg(
                msg: ::r2r::sensor_msgs::msg::PointCloud2,
            ) -> ::ros_pointcloud2::PointCloud2Msg {
                ::ros_pointcloud2::PointCloud2Msg {
                    header: ::ros_pointcloud2::ros::HeaderMsg {
                        seq: 0,
                        stamp: time_to_internal(msg.header.stamp),
                        frame_id: msg.header.frame_id,
                    },
                    dimensions: ::ros_pointcloud2::CloudDimensions {
                        width: msg.width,
                        height: msg.height,
                    },
                    fields: msg
                        .fields
                        .into_iter()
                        .map(|field| ::ros_pointcloud2::ros::PointFieldMsg {
                            name: field.name,
                            offset: field.offset,
                            datatype: field.datatype,
                            count: field.count,
                        })
                        .collect(),
                    endian: if msg.is_bigendian {
                        ::ros_pointcloud2::Endian::Big
                    } else {
                        ::ros_pointcloud2::Endian::Little
                    },
                    point_step: msg.point_step,
                    row_step: msg.row_step,
                    data: msg.data,
                    dense: if msg.is_dense {
                        ::ros_pointcloud2::Denseness::Dense
                    } else {
                        ::ros_pointcloud2::Denseness::Sparse
                    },
                }
            }

            pub fn from_pointcloud2_msg(
                msg: ::ros_pointcloud2::PointCloud2Msg,
            ) -> ::r2r::sensor_msgs::msg::PointCloud2 {
                ::r2r::sensor_msgs::msg::PointCloud2 {
                    header: ::r2r::std_msgs::msg::Header {
                        stamp: ::r2r::builtin_interfaces::msg::Time {
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
                        .map(|field| ::r2r::sensor_msgs::msg::PointField {
                            name: field.name,
                            offset: field.offset,
                            datatype: field.datatype,
                            count: field.count,
                        })
                        .collect(),
                    is_bigendian: matches!(msg.endian, ::ros_pointcloud2::Endian::Big),
                    point_step: msg.point_step,
                    row_step: msg.row_step,
                    data: msg.data,
                    is_dense: matches!(msg.dense, ::ros_pointcloud2::Denseness::Dense),
                }
            }

            pub fn time_to_internal(
                time: ::r2r::builtin_interfaces::msg::Time,
            ) -> ::ros_pointcloud2::ros::TimeMsg {
                ::ros_pointcloud2::ros::TimeMsg {
                    sec: time.sec,
                    nanosec: time.nanosec,
                }
            }

            pub fn time_from_internal(
                time: ::ros_pointcloud2::ros::TimeMsg,
            ) -> ::r2r::builtin_interfaces::msg::Time {
                ::r2r::builtin_interfaces::msg::Time {
                    sec: time.sec,
                    nanosec: time.nanosec,
                }
            }
        }
    };
}

#[macro_export]
macro_rules! impl_pointcloud2_for_ros2_interfaces_jazzy_serde {
    () => {
        pub mod impl_ros2_interfaces_jazzy_serde {
            pub fn to_pointcloud2_msg(
                msg: ::ros2_interfaces_jazzy_serde::sensor_msgs::msg::PointCloud2,
            ) -> ::ros_pointcloud2::PointCloud2Msg {
                ::ros_pointcloud2::PointCloud2Msg {
                    header: ::ros_pointcloud2::ros::HeaderMsg {
                        seq: 0,
                        stamp: time_to_internal(msg.header.stamp),
                        frame_id: msg.header.frame_id,
                    },
                    dimensions: ::ros_pointcloud2::CloudDimensions {
                        width: msg.width,
                        height: msg.height,
                    },
                    fields: msg
                        .fields
                        .into_iter()
                        .map(|field| ::ros_pointcloud2::ros::PointFieldMsg {
                            name: field.name,
                            offset: field.offset,
                            datatype: field.datatype,
                            count: field.count,
                        })
                        .collect(),
                    endian: if msg.is_bigendian {
                        ::ros_pointcloud2::Endian::Big
                    } else {
                        ::ros_pointcloud2::Endian::Little
                    },
                    point_step: msg.point_step,
                    row_step: msg.row_step,
                    data: msg.data,
                    dense: if msg.is_dense {
                        ::ros_pointcloud2::Denseness::Dense
                    } else {
                        ::ros_pointcloud2::Denseness::Sparse
                    },
                }
            }

            pub fn from_pointcloud2_msg(
                msg: ::ros_pointcloud2::PointCloud2Msg,
            ) -> ::ros2_interfaces_jazzy_serde::sensor_msgs::msg::PointCloud2 {
                ::ros2_interfaces_jazzy_serde::sensor_msgs::msg::PointCloud2 {
                    header: ::ros2_interfaces_jazzy_serde::std_msgs::msg::Header {
                        stamp: ::ros2_interfaces_jazzy_serde::builtin_interfaces::msg::Time {
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
                            |field| ::ros2_interfaces_jazzy_serde::sensor_msgs::msg::PointField {
                                name: field.name,
                                offset: field.offset,
                                datatype: field.datatype,
                                count: field.count,
                            },
                        )
                        .collect(),
                    is_bigendian: matches!(msg.endian, ::ros_pointcloud2::Endian::Big),
                    point_step: msg.point_step,
                    row_step: msg.row_step,
                    data: msg.data,
                    is_dense: matches!(msg.dense, ::ros_pointcloud2::Denseness::Dense),
                }
            }

            pub fn time_to_internal(
                time: ::ros2_interfaces_jazzy_serde::builtin_interfaces::msg::Time,
            ) -> ::ros_pointcloud2::ros::TimeMsg {
                ::ros_pointcloud2::ros::TimeMsg {
                    sec: time.sec,
                    nanosec: time.nanosec,
                }
            }

            pub fn time_from_internal(
                time: ::ros_pointcloud2::ros::TimeMsg,
            ) -> ::ros2_interfaces_jazzy_serde::builtin_interfaces::msg::Time {
                ::ros2_interfaces_jazzy_serde::builtin_interfaces::msg::Time {
                    sec: time.sec,
                    nanosec: time.nanosec,
                }
            }
        }
    };
}

#[macro_export]
macro_rules! impl_pointcloud2_for_ros2_interfaces_jazzy_rkyv {
    () => {
        pub mod impl_ros2_interfaces_jazzy_rkyv {
            pub fn to_pointcloud2_msg(
                msg: ::ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointCloud2,
            ) -> ::ros_pointcloud2::PointCloud2Msg {
                ::ros_pointcloud2::PointCloud2Msg {
                    header: ::ros_pointcloud2::ros::HeaderMsg {
                        seq: 0,
                        stamp: time_to_internal(msg.header.stamp),
                        frame_id: msg.header.frame_id,
                    },
                    dimensions: ::ros_pointcloud2::CloudDimensions {
                        width: msg.width,
                        height: msg.height,
                    },
                    fields: msg
                        .fields
                        .into_iter()
                        .map(|field| ::ros_pointcloud2::ros::PointFieldMsg {
                            name: field.name,
                            offset: field.offset,
                            datatype: field.datatype,
                            count: field.count,
                        })
                        .collect(),
                    endian: if msg.is_bigendian {
                        ::ros_pointcloud2::Endian::Big
                    } else {
                        ::ros_pointcloud2::Endian::Little
                    },
                    point_step: msg.point_step,
                    row_step: msg.row_step,
                    data: msg.data,
                    dense: if msg.is_dense {
                        ::ros_pointcloud2::Denseness::Dense
                    } else {
                        ::ros_pointcloud2::Denseness::Sparse
                    },
                }
            }

            pub fn from_pointcloud2_msg(
                msg: ::ros_pointcloud2::PointCloud2Msg,
            ) -> ::ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointCloud2 {
                ::ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointCloud2 {
                    header: ::ros2_interfaces_jazzy_rkyv::std_msgs::msg::Header {
                        stamp: ::ros2_interfaces_jazzy_rkyv::builtin_interfaces::msg::Time {
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
                            |field| ::ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointField {
                                name: field.name,
                                offset: field.offset,
                                datatype: field.datatype,
                                count: field.count,
                            },
                        )
                        .collect(),
                    is_bigendian: matches!(msg.endian, ::ros_pointcloud2::Endian::Big),
                    point_step: msg.point_step,
                    row_step: msg.row_step,
                    data: msg.data,
                    is_dense: matches!(msg.dense, ::ros_pointcloud2::Denseness::Dense),
                }
            }

            pub fn time_to_internal(
                time: ::ros2_interfaces_jazzy_rkyv::builtin_interfaces::msg::Time,
            ) -> ::ros_pointcloud2::ros::TimeMsg {
                ::ros_pointcloud2::ros::TimeMsg {
                    sec: time.sec,
                    nanosec: time.nanosec,
                }
            }

            pub fn time_from_internal(
                time: ::ros_pointcloud2::ros::TimeMsg,
            ) -> ::ros2_interfaces_jazzy_rkyv::builtin_interfaces::msg::Time {
                ::ros2_interfaces_jazzy_rkyv::builtin_interfaces::msg::Time {
                    sec: time.sec,
                    nanosec: time.nanosec,
                }
            }
        }
    };
}

#[macro_export]
macro_rules! impl_pointcloud2_for_rosrust {
    () => {
        pub mod impl_rosrust {
            pub fn to_pointcloud2_msg(
                msg: rosrust_msg::sensor_msgs::PointCloud2,
            ) -> ::ros_pointcloud2::PointCloud2Msg {
                ::ros_pointcloud2::PointCloud2Msg {
                    header: ::ros_pointcloud2::ros::HeaderMsg {
                        seq: msg.header.seq,
                        stamp: time_to_internal(msg.header.stamp),
                        frame_id: msg.header.frame_id,
                    },

                    dimensions: ::ros_pointcloud2::CloudDimensions {
                        width: msg.width,
                        height: msg.height,
                    },
                    fields: msg
                        .fields
                        .into_iter()
                        .map(|field| ::ros_pointcloud2::ros::PointFieldMsg {
                            name: field.name,
                            offset: field.offset,
                            datatype: field.datatype,
                            count: field.count,
                        })
                        .collect(),
                    endian: if msg.is_bigendian {
                        ::ros_pointcloud2::Endian::Big
                    } else {
                        ::ros_pointcloud2::Endian::Little
                    },
                    point_step: msg.point_step,
                    row_step: msg.row_step,
                    data: msg.data,
                    dense: if msg.is_dense {
                        ::ros_pointcloud2::Denseness::Dense
                    } else {
                        ::ros_pointcloud2::Denseness::Sparse
                    },
                }
            }

            /// Convert an internal `PointCloud2Msg` into `rosrust_msg::sensor_msgs::PointCloud2`.
            pub fn from_pointcloud2_msg(
                msg: ::ros_pointcloud2::PointCloud2Msg,
            ) -> rosrust_msg::sensor_msgs::PointCloud2 {
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
                    is_bigendian: if msg.endian == ::ros_pointcloud2::Endian::Big {
                        true
                    } else {
                        false
                    },
                    point_step: msg.point_step,
                    row_step: msg.row_step,
                    data: msg.data,
                    is_dense: if msg.dense == ::ros_pointcloud2::Denseness::Dense {
                        true
                    } else {
                        false
                    },
                }
            }

            pub fn time_to_internal(time: rosrust::Time) -> ::ros_pointcloud2::ros::TimeMsg {
                ::ros_pointcloud2::ros::TimeMsg {
                    sec: time.sec as i32,
                    nanosec: time.nsec,
                }
            }

            pub fn time_from_internal(time: ::ros_pointcloud2::ros::TimeMsg) -> rosrust::Time {
                rosrust::Time {
                    sec: time.sec as u32,
                    nsec: time.nanosec,
                }
            }
        }
    };
}

#[macro_export]
macro_rules! impl_pointcloud2_for_rclrs {
    () => {
        pub mod impl_rclrs {
            pub fn to_pointcloud2_msg(
                msg: sensor_msgs::msg::PointCloud2,
            ) -> ::ros_pointcloud2::PointCloud2Msg {
                ::ros_pointcloud2::PointCloud2Msg {
                    header: ::ros_pointcloud2::ros::HeaderMsg {
                        seq: 0,
                        stamp: time_to_internal(msg.header.stamp),
                        frame_id: msg.header.frame_id,
                    },
                    dimensions: ::ros_pointcloud2::CloudDimensions {
                        width: msg.width,
                        height: msg.height,
                    },
                    fields: msg
                        .fields
                        .into_iter()
                        .map(|field| ::ros_pointcloud2::ros::PointFieldMsg {
                            name: field.name,
                            offset: field.offset,
                            datatype: field.datatype,
                            count: field.count,
                        })
                        .collect(),
                    endian: if msg.is_bigendian {
                        ::ros_pointcloud2::Endian::Big
                    } else {
                        ::ros_pointcloud2::Endian::Little
                    },
                    point_step: msg.point_step,
                    row_step: msg.row_step,
                    data: msg.data,
                    dense: if msg.is_dense {
                        ::ros_pointcloud2::Denseness::Dense
                    } else {
                        ::ros_pointcloud2::Denseness::Sparse
                    },
                }
            }

            pub fn from_pointcloud2_msg(
                msg: ::ros_pointcloud2::PointCloud2Msg,
            ) -> sensor_msgs::msg::PointCloud2 {
                sensor_msgs::msg::PointCloud2 {
                    header: std_msgs::msg::Header {
                        stamp: builtin_interfaces::msg::Time {
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
                        .map(|field| sensor_msgs::msg::PointField {
                            name: field.name,
                            offset: field.offset,
                            datatype: field.datatype,
                            count: field.count,
                        })
                        .collect(),
                    is_bigendian: matches!(msg.endian, ::ros_pointcloud2::Endian::Big),
                    point_step: msg.point_step,
                    row_step: msg.row_step,
                    data: msg.data,
                    is_dense: matches!(msg.dense, ::ros_pointcloud2::Denseness::Dense),
                }
            }

            pub fn time_to_internal(
                time: builtin_interfaces::msg::Time,
            ) -> ::ros_pointcloud2::ros::TimeMsg {
                ::ros_pointcloud2::ros::TimeMsg {
                    sec: time.sec,
                    nanosec: time.nanosec,
                }
            }

            pub fn time_from_internal(
                time: ::ros_pointcloud2::ros::TimeMsg,
            ) -> builtin_interfaces::msg::Time {
                builtin_interfaces::msg::Time {
                    sec: time.sec,
                    nanosec: time.nanosec,
                }
            }
        }
    };
}
