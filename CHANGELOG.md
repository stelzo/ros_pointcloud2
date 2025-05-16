# Changelog

## v0.5.2 -> v0.6.0
- All derived points now require `Copy`. This allows `try_from_iter` and `try_from_vec` to not require ownership for the parameter.
- Renamed `rpcl2` derive attribute to `ros`. E.g. `#[rpcl2(rename("test"))]` -> `#[ros(rename("test"))]`.
- Feature `ros2-interfaces-jazzy` moved to `ros2-interfaces-jazzy-serde` to keep up-to-date with the latest ros2-client.
- Dropping rclrs support until their message generation strategy is finished and the integration can be made easier. The current implementation for is bad for long term maintenance. You can keep using `v0.5.2_rclrs` instead.
- Undeprecates xyz() and deprecates xyz_f32() and xyz_f64(). Use casting instead. Types should not be part of a function name.
- Adds the features `ros2-interfaces-jazzy-serde` for ros2-client support and `ros2-interfaces-jazzy-rkyv` for ROS-like systems that use rkyv for (de)serialization.

## v0.5.1 -> v0.5.2
- Fixes rosrust integration due to a yanked transitive dependency.
- Adds Serialization and Deserialization of public structs with new feature serde.
- Deprecates xyz() functions in favor of explicit types -> xyz_f32() | xyz_f64()

## v0.5.0 -> v0.5.1

- Fixes a bug, where the conversion of larger to smaller types results in a false buffer interpretation.

## v0.5.0-rc.3 -> v0.5.0

- `PointConvertible` trait is now `unsafe` since the offset is used for raw memory access, where safety can not be guaranteed by the compiler.
- Fixes clippy on nightly.
- Fixes a bug when attempting to write larger types than available in the message. This now results in a `ExhaustedSource` error.
- Adds `repr(C)` to docs where custom conversions are explained to encourage best practices for raw type descriptions.

## v0.5.0-rc.2 -> v0.5.0-rc.3

- Bump r2r to 0.9.
- Fixed building in `no_std` environments.
- Removed `expect` calls.

## v0.5.0-rc.1 -> v0.5.0-rc.2

- `PointConvertible` now includes the information for `TypeLayout` and `Fields`, which reduces boilerplate code for custom points. The respective derive macro is updated to work with the updated trait.
- `_vec` functions now work without the `derive` feature and thus are always available.
- The `derive` feature now is disabled by default but it is still strongly recommended for custom points to avoid layout errors. This also makes procmacro dependencies optional for the functionality, since every conversion can be called without them.
- The alignment of all predefined points is increased for SSE optimization and optimized copies between C++ PCL and Rust.

## v0.4.0 -> v0.5.0-rc.1

Most of the library is rewritten to be simpler and more expandable while adding mostly performance focused features to motivate the breaking changes.

## Why?

The previous concept of the `Convert` struct for both directions of conversion leads to use cases, where the behavior is unexpected for the user.

## What changed?

Nearly every public function is changed to be easier to work with and faster at runtime and adaptable for different scenarios.

It is easier to update to the new version by looking at it as a new crate and starting with the documentation.

### Breaking

- Switched from FallibleIterator to Iterator, since all possible errors are checked before returning the type.
- Renamed `pcl_utils` module to `points` to make the module more a general toolkit and less a PCL compatibility layer.
- Renamed `ros_types` module to `ros` since types are inferred.
- Removed `Convert` struct. The `PointCloud2Msg` now directly offers `try_from_iter` and `try_into_iter` functions.
- Dimensions and PointMeta in `PointConvertible` are merged to types, that are mostly deducted to minimize the needed code (see the docs for a custom point example).
- Conversions can not fail per point in `PointConvertible`, so TryFrom -> From.
- RGB is now stored as a union to allow direct copy with `_vec` functions without packing it manually per point. There are setters and getters for safety and usability.
- ros_types::TimeMsg (now ros::TimeMsg) uses `nanosec` instead of `nsec` naming to conform with ROS2. This also removes the type alias for `rosrust`. Instead, there is a `impl From<rosrust::Time> for TimeMsg` now.

### Added features

- `[from|into]_vec` enabled by derive for memory heavy applications (enabled by default).
- `[from|into]_par_iter` enabled by rayon for processing heavy applications.
- `derive` macros for minimizing code for custom point conversions (enabled by default).
- `xyz()` getter enabled by nalgebra for all predefined points.
- `prelude::*` re-export module for quickly importing common types in point cloud processing projects.
- Lightweight: `no_std` compatibility for `iter` conversions and dependency-free builds when only needing `iter` conversions.
- Overall speed up thanks to more vectorizable iterator conversions and the new `vec` and `par_iter` functions. See the [comparison with PCL](https://github.com/stelzo/ros_pcl_conv_bench) for more info.
- More type deduction in public facing functions, leading to less code for the user of the library.
- More documentation.
