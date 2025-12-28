<p align="center">
  <h3 align="center">ROS PointCloud2</h3>
  <p align="center">A PointCloud2 message conversion library.</p>
  <p align="center"><a href="https://crates.io/crates/ros_pointcloud2"><img src="https://img.shields.io/crates/v/ros_pointcloud2.svg" alt=""></a> <a href="https://github.com/stelzo/ros_pointcloud2/tree/main/tests"><img src="https://github.com/stelzo/ros_pointcloud2/actions/workflows/tests.yml/badge.svg" alt=""></a>
  </p>
</p>

## NOTE: This crate is under active development for v1.0.0 release. See the docs.rs page for the latest stable documentation.

ros_pointcloud2 uses its own type for the message `PointCloud2Msg` to keep the library framework agnostic. ROS1 and ROS2 are supported with impl macros for common crates.

Get started with the example below, check out the other use cases in the `examples` folder or see the [Documentation](https://docs.rs/ros_pointcloud2/latest/ros_pointcloud2/) for a complete guide.

## Quickstart

```toml
[dependencies]
ros_pointcloud2 = "1"
```

```rust
use ros_pointcloud2::prelude::*;

// PointXYZ (and many others) are provided by the crate.
let cloud_points = vec![
  PointXYZI::new(9.6, 42.0, -6.2, 0.1),
  PointXYZI::new(46.0, 5.47, 0.5, 0.1),
];

let out_msg = PointCloud2Msg::try_from_slice(&cloud_points).unwrap();

// Add your ROS crate impl macro in your crate to enable conversions.
// For example, for r2r:
// ros_pointcloud2::impl_pointcloud2_for_r2r!();

// Convert to your ROS crate message type.
// let msg: PointCloud2 = crate::impl_r2r::from_pointcloud2_msg(out_msg);
// Publish ...

// ... now incoming from a topic.
// let in_msg: PointCloud2Msg = crate::impl_r2r::to_pointcloud2_msg(msg);
let in_msg = out_msg;

let processed_cloud = in_msg.try_into_iter().unwrap()
  .map(|point: PointXYZI| { // Define the type you want to map the data to.
    // Access the data like a normal struct.
    PointXYZI::new(point.x, point.y, point.z, 0.1)
  })
  .collect::<Vec<_>>();
```

## Integrations

There are currently 4 integrations for common ROS crates. We tested them on the following distros.

- [rosrust](https://github.com/adnanademovic/rosrust)
  - Noetic
- [r2r](https://github.com/sequenceplanner/r2r)
  - Galactic
  - Humble
  - Jazzy
- [rclrs](https://github.com/ros2-rust/ros2_rust)
  - Humble
  - Iron
- [ros2-client](https://github.com/Atostek/ros2-client.git)
  - Jazzy

You can use `rosrust`, `r2r` or `ros2-client` by prefixing the designated macro somewhere in your scope after adding your ros crate to your `Cargo.toml`.

```rust
// r2r
ros_pointcloud2::impl_pointcloud2_for_r2r!();
// rosrust
ros_pointcloud2::impl_pointcloud2_for_rosrust!();
// ros2-client
ros_pointcloud2::impl_pointcloud2_for_ros2_interfaces_jazzy_serde!();
// ½[experimental] rclrs
ros_pointcloud2::impl_pointcloud2_for_rclrs!();

// nalgebra support for getting points as nalgebra types
ros_pointcloud2::impl_pointxyz_for_nalgebra!();
// then you can use the conversions with your current nalgebra setup
use ros_pointcloud2::points::PointXYZI;
use ros_pointcloud2::impl_nalgebra::AsNalgebra;
let p_xyzi = PointXYZI::new(4.0, 5.0, 6.0, 7.0);
assert_eq!(AsNalgebra::xyz(&p_xyzi), Point3::new(4.0, 5.0, 6.0));
```

### rclrs (ros2_rust)

`rcrls` is not as easy to use as other crates due to its deep integration with ROS2. For v1.0.0, I tried to setup CI tests but was not able to compile the most recent `rclrs`. This does not affect the general API of this crate so it is here as an untested option for users who can get `rclrs` working in their environment. If you can get it working, please open a PR or issue to help others.


For all others, there is a special tag `v0.5.2_rclrs` that contains a tested version from some time ago that you can use as follows:

```toml
[dependencies]
ros_pointcloud2 = { git = "https://github.com/stelzo/ros_pointcloud2", tag = "v0.5.2_rclrs" }
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
