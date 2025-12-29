^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for the ros_pointcloud2 crate
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

v1.0.0
---------------

- ``try_from_iter`` and ``try_into_iter`` now accept references to iterators.
- ``try_from_vec`` takes ownership again but tries to reuse the memory when possible.
- Adds ``try_from_vec_strict`` that hard fails on unsupported layouts.
- Adds ``try_into_slice`` that tries to create a zero-copy slice view, falls back to owned ``Vec`` if needed.
- Adds ``try_into_slice_strict`` that hard fails on unsupported layouts.
- Adds ``try_from_slice``, which creates a message from a slice with minimal copy.
- Adds full support for ``rclrs``.
- Adds full support for ``roslibrust`` for ROS1 and ROS2.
- Renamed the ``rename`` attribute to ``remap`` for custom field names.
- Renamed the ``MsgConversionError`` enum to ``ConversionError``.
- Adds the ``strict-type-check`` feature that enforces strict validation of field datatypes during conversion; ``RGB <-> f32`` compatibility is preserved for legacy packed RGB fields. This feature is enabled by default. When ``strict-type-check`` is enabled, reading a mismatched type returns a ``ConversionError::TypeMismatch`` at runtime to allow on-the-fly type matching for the user.
- The ``ConversionError`` variants are the same now on ``std`` and ``no_std`` builds.
- Fixed copying structured pointclouds from ``Vec`` with same endianness.
- New feature ``rkyv`` for (de)serialization of ``PointCloud2Msg`` using ``rkyv``. This allows zero-copy deserialization in ROS-like systems that use ``rkyv`` for message (de)serialization.
- Many internal optimizations for runtime speed and code maintainability; the main benefit is the removed need for allocations in the hot path of conversions.
- Bump MSRV to 1.87.
- Adds various tests for edge cases.

v0.6.0
----------------

- All derived points now require ``Copy``. This allows ``try_from_iter`` and ``try_from_vec`` to not require ownership for the parameter.
- Renamed ``rpcl2`` derive attribute to ``ros``. E.g. ``#[rpcl2(rename("test"))]`` -> ``#[ros(rename("test"))]``.
- Feature ``ros2-interfaces-jazzy`` moved to ``ros2-interfaces-jazzy-serde`` to keep up-to-date with the latest ros2-client.
- Dropping ``rclrs`` support until their message generation strategy is finished and the integration can be made easier. The current implementation is poor for long-term maintenance. You can keep using ``v0.5.2_rclrs`` instead.
- Undeprecates ``xyz()`` and deprecates ``xyz_f32()`` and ``xyz_f64()``. Use casting instead. Types should not be part of a function name.
- Adds the features ``ros2-interfaces-jazzy-serde`` for ros2-client support and ``ros2-interfaces-jazzy-rkyv`` for ROS-like systems that use ``rkyv`` for (de)serialization.

v0.5.2
----------------

- Fixes rosrust integration due to a yanked transitive dependency.
- Adds serialization and deserialization of public structs with the new feature ``serde``.
- Deprecates ``xyz()`` functions in favor of explicit types: ``xyz_f32()`` | ``xyz_f64()``.

v0.5.1
----------------

- Fixes a bug where the conversion of larger to smaller types resulted in a false buffer interpretation.

v0.5.0
---------------------

- ``PointConvertible`` trait is now ``unsafe`` since the offset is used for raw memory access, where safety cannot be guaranteed by the compiler.
- Fixes ``clippy`` on nightly.
- Fixes a bug when attempting to write larger types than available in the message. This now results in an ``ExhaustedSource`` error.
- Adds ``repr(C)`` to docs where custom conversions are explained to encourage best practices for raw type descriptions.

v0.5.0-rc.3
-------------------------

- Bump ``r2r`` to 0.9.
- Fixed building in ``no_std`` environments.
- Removed ``expect`` calls.

v0.5.0-rc.2
-------------------------

- ``PointConvertible`` now includes the information for ``TypeLayout`` and ``Fields``, which reduces boilerplate code for custom points. The respective derive macro is updated to work with the updated trait.
- ``_vec`` functions now work without the ``derive`` feature and thus are always available.
- The ``derive`` feature is disabled by default but remains recommended for custom points to avoid layout errors. This also makes proc-macro dependencies optional for the functionality, since every conversion can be called without them.
- The alignment of all predefined points is increased for SSE optimization and optimized copies between C++ PCL and Rust.

v0.5.0-rc.1
--------------------

Most of the library was rewritten to be simpler and more expandable while adding performance-focused features that motivated the breaking changes.

Why?
The previous concept of the ``Convert`` struct for both directions of conversion led to use cases where behavior was unexpected for the user.

What changed?
Nearly every public function was changed to be easier to work with and faster at runtime and adaptable for different scenarios.

Breaking
- Switched from ``FallibleIterator`` to ``Iterator``, since all possible errors are checked before returning the type.
- Renamed ``pcl_utils`` module to ``points`` to make the module a more general toolkit and less a PCL compatibility layer.
- Renamed ``ros_types`` module to ``ros`` since types are inferred.
- Removed ``Convert`` struct. The ``PointCloud2Msg`` now directly offers ``try_from_iter`` and ``try_into_iter`` functions.
- ``Dimensions`` and ``PointMeta`` in ``PointConvertible`` are merged to types that are mostly deduced to minimize the needed code (see the docs for a custom point example).
- Conversions cannot fail per point in ``PointConvertible``, so ``TryFrom`` -> ``From``.
- ``RGB`` is now stored as a union to allow direct copy with ``_vec`` functions without packing it manually per point. There are setters and getters for safety and usability.
- ``ros::TimeMsg`` uses ``nanosec`` instead of ``nsec`` naming to conform with ROS2. This also removes the type alias for ``rosrust``. Instead, there is an ``impl From<rosrust::Time> for TimeMsg`` now.

Added features
- ``[from|into]_vec`` enabled by derive for memory-heavy applications (enabled by default).
- ``[from|into]_par_iter`` enabled by ``rayon`` for processing-heavy applications.
- ``derive`` macros for minimizing code for custom point conversions (enabled by default).
- ``xyz()`` getter enabled by ``nalgebra`` for all predefined points.
- ``prelude::*`` re-export module for quickly importing common types in point cloud processing projects.
- Lightweight: ``no_std`` compatibility for ``iter`` conversions and dependency-free builds when only needing ``iter`` conversions.
- Overall speed up thanks to more vectorizable iterator conversions and the new ``vec`` and ``par_iter`` functions. See the comparison with PCL: https://github.com/stelzo/ros_pcl_conv_bench
- More type deduction in public-facing functions, leading to less code for the user of the library.
- More documentation.