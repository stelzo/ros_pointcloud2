## !! Note !!
This library is currently in development for v1.0.0, for the documentation of v0.4.0 on crates.io, visit the [docs](https://docs.rs/ros_pointcloud2/0.4.0/ros_pointcloud2/).

<p align="center">
  <h3 align="center">ROS PointCloud2</h3>
  <p align="center">A complete and versatile abstraction of PointCloud2.</p>
  <p align="center"><a href="https://crates.io/crates/ros_pointcloud2"><img src="https://img.shields.io/crates/v/ros_pointcloud2.svg" alt=""></a> <a href="https://github.com/stelzo/ros_pointcloud2/tree/main/tests"><img src="https://github.com/stelzo/ros_pointcloud2/actions/workflows/tests.yml/badge.svg" alt=""></a>
  </p>
</p>

To keep the crate a general purpose library for the problem, it uses its own type for the message `PointCloud2Msg`.
ROS1 and ROS2 is supported with feature flags.

Get started with the example below, check out the other use cases in the `examples` folder or see the [Documentation](https://docs.rs/ros_pointcloud2/1.0.0/ros_pointcloud2/) for a complete guide.

## Quickstart

The following example uses the iterator APIs. For memory bound pipelines, you may also try the `try_from_vec` or `try_into_vec` functions by enabling the `derive` feature.

```rust
use ros_pointcloud2::prelude::*;

// PointXYZ (and many others) are provided by the crate.
let cloud_points = vec![
  PointXYZ::new(91.486, -4.1, 42.0001),
  PointXYZ::new(f32::MAX, f32::MIN, f32::MAX),
];

// Give the Vec or anything that implements `IntoIterator`.
let in_msg = PointCloud2Msg::try_from_iter(cloud_points).unwrap();

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
