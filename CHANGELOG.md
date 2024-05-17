# Changelog

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