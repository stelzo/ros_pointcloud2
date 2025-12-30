<p align="center">
  <h3 align="center">ROS PointCloud2</h3>
  <p align="center">A PointCloud2 message conversion library.</p>
  <p align="center"><a href="https://crates.io/crates/ros_pointcloud2"><img src="https://img.shields.io/crates/v/ros_pointcloud2.svg" alt=""></a> <a href="https://github.com/stelzo/ros_pointcloud2/tree/main/tests"><img src="https://github.com/stelzo/ros_pointcloud2/actions/workflows/tests.yml/badge.svg" alt=""></a>
  </p>
</p>

This library provides ergonomic and safe abstractions for the `PointCloud2` type, including conversions to and from prominent ROS libraries.

To remain framework-agnostic, the crate uses its own `PointCloud2Msg` type, ensuring compatibility across different ROS ecosystems.

Get started with the example below or visit the [documentation](https://docs.rs/ros_pointcloud2/1.0.0-rc.1/ros_pointcloud2/) for a complete guide.

## Quickstart

> [!NOTE]  
> We are transitioning to a new version (v1.0.0). Please open an issue if you encounter problems. The stable version remains v0.6.0. For the latest stable release, refer to the [v0.6.0 documentation](https://docs.rs/ros_pointcloud2/0.6.0/ros_pointcloud2/). Check the [changelog](./CHANGELOG.rst) for a full list of changes

```toml
[dependencies]
ros_pointcloud2 = "1.0.0-rc.1" # or use the old version 0.6
```

```rust
use ros_pointcloud2::prelude::*;

// PointXYZ (and many others) are provided by the crate.
let cloud_points = vec![
  PointXYZI::new(9.6, 42.0, -6.2, 0.1),
  PointXYZI::new(46.0, 5.47, 0.5, 0.1),
];

let out_msg = PointCloud2Msg::try_from_slice(&cloud_points).unwrap();

// Outside of your main
// impl_pointcloud2_for_r2r!();

// Convert to your ROS crate message type.
// let msg = impl_r2r::from_pointcloud2_msg(out_msg);
// Publish ...

// ... now incoming from a topic.
// let in_msg = impl_r2r::to_pointcloud2_msg(msg);
let in_msg = out_msg;

let processed_cloud = in_msg.try_into_iter().unwrap()
  .map(|point: PointXYZI| { // Define the type you want to map the data to.
    // Access the data like a normal struct.
    PointXYZI::new(point.x, point.y, point.z, 0.1)
  })
  .collect::<Vec<_>>();
```

## Support for ROS client libraries

There is currently support for the following ROS libraries.

| Library | Macro | ROS Version |
| :--- | :--- | :--- |
| [rclrs](https://docs.rs/rclrs/latest/rclrs/) | `impl_pointcloud2_for_rclrs!()` | ROS 2 |
| [r2r](https://docs.rs/r2r/latest/r2r/) | `impl_pointcloud2_for_r2r!()` | ROS 2 |
| [ros2-client](https://docs.rs/ros2-client/latest/ros2_client/) | `impl_pointcloud2_for_ros2_interfaces_jazzy_serde!()` | ROS 2 |
| [rosrust](https://docs.rs/rosrust/latest/rosrust/) | `impl_pointcloud2_for_rosrust!()` | ROS 1 |
| [roslibrust](https://docs.rs/roslibrust/latest/roslibrust/) | `impl_pointcloud2_for_roslibrust_ros1!(crate)` | ROS 1 |
| [roslibrust](https://docs.rs/roslibrust/latest/roslibrust/) | `impl_pointcloud2_for_roslibrust_ros2!(crate)` | ROS 2 |

Note that the macros for `roslibrust` need the root path where messages are included via `include!` as a parameter.

The macros are tested on various distros with CI. Since the message definition itself is static for a long time now, this crate should work on all ROS distros for as long as the respective ROS library or framework does not change its message generation pipeline.

You can use this library with your ROS crate by prefixing the designated macro somewhere in your scope after adding your ROS crate to your `Cargo.toml`.

Also, add the following dependencies to your `package.xml` (if your library of choice uses them) to ensure all message types are correctly resolved.

```xml
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>builtin_interfaces</depend>
```

Please open an issue or PR if you need support for other ROS libraries.

There is also nalgebra support to convert common point types to the nalgebra `Point3` type.

```rust
ros_pointcloud2::impl_pointxyz_for_nalgebra!();

// then you can use the conversions with your current nalgebra setup
use ros_pointcloud2::points::PointXYZI;
use ros_pointcloud2::impl_nalgebra::AsNalgebra;
let p_xyzi = PointXYZI::new(4.0, 5.0, 6.0, 7.0);
assert_eq!(AsNalgebra::xyz(&p_xyzi), nalgebra::Point3::new(4.0, 5.0, 6.0));
```

## Performance

This library is designed with Rust's zero-cost philosophy in mind. It minimizes overhead by avoiding unnecessary allocations during conversion and leveraging zero-copy iterators and memory views for point data when possible.

In many cases, this results in a significant speedup compared to standard PointCloudLibrary (PCL) conversions. However, the exact factor depends on your specific data layout and system.

Unlike many other implementations, this crate provides full spec compliance for the PointCloud2 message format, including correct handling of endianness and diverse field layouts.

For detailed benchmarks, see the [ros_pcl_conv_bench](https://github.com/stelzo/ros_pcl_conv_bench) repository.

To minimize the conversion overhead in general, always use the function that best fits your use case.

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
