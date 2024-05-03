<p align="center">
  <h3 align="center">ROS PointCloud2</h3>
  <p align="center">Customizable conversions to and from the PointCloud2 ROS message.</p>
  <p align="center"><a href="https://crates.io/crates/ros_pointcloud2"><img src="https://img.shields.io/crates/v/ros_pointcloud2.svg" alt=""></a> <a href="https://github.com/stelzo/ros_pointcloud2/tree/main/tests"><img src="https://github.com/stelzo/ros_pointcloud2/actions/workflows/tests.yml/badge.svg" alt=""></a>
  </p>
</p>

A complete implementation of `sensor_msgs/PointCloud2` conversions with focus on ease of use and maximum throughput.

Providing an easy to use, generics defined, point-wise iterator abstraction over the byte buffer in `PointCloud2` to minimize iterations in your processing pipeline.

To keep the crate a general purpose library for the problem, it uses its own type for the message `PointCloud2Msg`. ROS1 and ROS2 support is added with feature flags.

## Performance

The library compares the similarity of the ROS message point type with your given Point, allowing it to select the fastest strategy.

Since PCL is the most used library for this work in C++, the predefined types in ros_pointcloud2 use C padding to be exact copies for maximum performance between C++ and Rust nodes at the cost of a slightly larger message sizes.

Since ros_pointcloud2 optimizes the performance dynamically, it typically outperforms PCL conversions in all tasks.

## Quickstart (Iterator)
```rust
use ros_pointcloud2::{PointCloud2Msg, pcl_utils::PointXYZ};

// Your points (here using a predefined type PointXYZ).
let cloud_points = vec![
  PointXYZ {x: 91.486, y: -4.1, z: 42.0001,},
  PointXYZ {x: f32::MAX, y: f32::MIN, z: f32::MAX,},
];

// Give the Vec or anything that implements `IntoIterator`.
let in_msg = PointCloud2Msg::try_from_iterable(cloud_points).unwrap();

// Convert the ROS crate message type, we will use r2r here.
// let msg: r2r::sensor_msgs::msg::PointCloud2 = in_msg.into();
// Publish ...
// ... now incoming from a topic.
// let in_msg: PointCloud2Msg = msg.into();

let new_pcl = in_msg.try_into_iter().unwrap()
  .map(|point: PointXYZ| { // Define the type of point here.
    // Some logic here ...

      point
    })
    .collect::<Vec<_>>();
```

## Integrations

There are currently 3 integrations for common ROS crates.
- [rosrust_msg](https://github.com/adnanademovic/rosrust)
  - [![Tests](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rosrust_noetic.yml/badge.svg)](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rosrust_noetic.yml)
- [r2r_msg](https://github.com/sequenceplanner/r2r)
  - [![Tests](https://github.com/stelzo/ros_pointcloud2/actions/workflows/r2r_galactic.yml/badge.svg)](https://github.com/stelzo/ros_pointcloud2/actions/workflows/r2r_galactic.yml)
  - [![Tests](https://github.com/stelzo/ros_pointcloud2/actions/workflows/r2r_humble.yml/badge.svg)](https://github.com/stelzo/ros_pointcloud2/actions/workflows/r2r_humble.yml)
  - [![Tests](https://github.com/stelzo/ros_pointcloud2/actions/workflows/r2r_iron.yml/badge.svg)](https://github.com/stelzo/ros_pointcloud2/actions/workflows/r2r_iron.yml)
- [rclrs_msg](https://github.com/ros2-rust/ros2_rust)
  - [![Tests](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rclrs_humble.yml/badge.svg)](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rclrs_humble.yml)
  - [![Tests](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rclrs_iron.yml/badge.svg)](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rclrs_iron.yml)

You can use `rosrust` and `r2r` by enabling the respective feature:
```toml
[dependencies]
ros_pointcloud2 = { version = "*", features = ["r2r_msg"]}
# or
ros_pointcloud2 = { version = "*", features = ["rosrust_msg"]}
```

### rclrs (ros2_rust)
Features do not work properly with `rcrls` because the messages are linked externally. You need to use tags instead:
```toml
[dependencies]
ros_pointcloud2 = { git = "https://github.com/stelzo/ros_pointcloud2", tag = "v0.4.0_rclrs" }
```
Also, indicate the following dependencies to your linker inside the `package.xml` of your package.
```xml
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>builtin_interfaces</depend>
```

Please open an issue or PR if you want to see support for other crates.

## License
[MIT](https://choosealicense.com/licenses/mit/)
