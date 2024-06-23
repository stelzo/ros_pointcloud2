<p align="center">
  <h3 align="center">ROS PointCloud2</h3>
  <p align="center">A PointCloud2 message conversion library.</p>
  <p align="center"><a href="https://crates.io/crates/ros_pointcloud2"><img src="https://img.shields.io/crates/v/ros_pointcloud2.svg" alt=""></a> <a href="https://github.com/stelzo/ros_pointcloud2/tree/main/tests"><img src="https://github.com/stelzo/ros_pointcloud2/actions/workflows/tests.yml/badge.svg" alt=""></a>
  </p>
</p>

ros_pointcloud2 uses its own type for the message `PointCloud2Msg` to keep the library framework agnostic. ROS1 and ROS2 are supported with feature flags.

Get started with the example below, check out the other use cases in the `examples` folder or see the [Documentation](https://docs.rs/ros_pointcloud2/0.5.0/) for a complete guide.

## Quickstart

```rust
use ros_pointcloud2::prelude::*;

// PointXYZ (and many others) are provided by the crate.
let cloud_points = vec![
  PointXYZI::new(91.486, -4.1, 42.0001, 0.1),
  PointXYZI::new(f32::MAX, f32::MIN, f32::MAX, f32::MIN),
];

let out_msg = PointCloud2Msg::try_from_vec(cloud_points).unwrap();

// Convert the ROS crate message type, we will use r2r here.
// let msg: r2r::sensor_msgs::msg::PointCloud2 = out_msg.into();
// Publish ...

// ... now incoming from a topic.
// let in_msg: PointCloud2Msg = msg.into();
let in_msg = out_msg;

let processed_cloud = in_msg.try_into_iter().unwrap()
  .map(|point: PointXYZ| { // Define the info you want to have from the Msg.
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
  - [![Tests](https://github.com/stelzo/ros_pointcloud2/actions/workflows/r2r_jazzy.yml/badge.svg)](https://github.com/stelzo/ros_pointcloud2/actions/workflows/r2r_jazzy.yml)
- [rclrs_msg](https://github.com/ros2-rust/ros2_rust)
  - [![Tests](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rclrs_humble.yml/badge.svg)](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rclrs_humble.yml)
  - [![Tests](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rclrs_iron.yml/badge.svg)](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rclrs_iron.yml)
  - [![Tests](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rclrs_jazzy.yml/badge.svg)](https://github.com/stelzo/ros_pointcloud2/actions/workflows/rclrs_jazzy.yml)

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
ros_pointcloud2 = { git = "https://github.com/stelzo/ros_pointcloud2", tag = "v0.5.0_rclrs" }
```

Also, indicate the following dependencies to your linker inside the `package.xml` of your package.

```xml
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>builtin_interfaces</depend>
```

Please open an issue or PR if you need other integrations.

## Performance

This library offers a speed up when compared to PointCloudLibrary (PCL) conversions but the specific factor depends heavily on the use case and system.
See [this repository](https://github.com/stelzo/ros_pcl_conv_bench) for a detailed benchmark.

For minimizing the conversion overhead in general, always use the functions that best fit your use case.

### License

<sup>
Licensed under either of <a href="LICENSE-APACHE">Apache License, Version
2.0</a> or <a href="LICENSE-MIT">MIT license</a> at your option.
</sup>

<br>

<sub>
Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in this crate by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
</sub>
