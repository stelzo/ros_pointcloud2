<p align="center">
  <h3 align="center">ROS PointCloud2</h3>
  <p align="center">Customizable conversions to and from the PointCloud2 ROS message.</p>
  <p align="center"><a href="https://crates.io/crates/ros_pointcloud2"><img src="https://img.shields.io/crates/v/ros_pointcloud2.svg" alt=""></a> <a href="https://github.com/stelzo/ros_pointcloud2/tree/main/tests"><img src="https://github.com/stelzo/ros_pointcloud2/actions/workflows/tests.yml/badge.svg" alt=""></a>
  </p>
</p>

Providing a memory efficient way for message conversion while allowing user defined types without the cost of iterations.
Instead of converting the entire cloud into a `Vec`, you get an iterable type that converts each point from the message on the fly.

To keep the crate a general purpose library for the problem and support ROS1 and ROS2, it uses its own type for the message `ros_types::PointCloud2Msg`.
```rust
use ros_pointcloud2::{
    fallible_iterator::FallibleIterator,
    pcl_utils::PointXYZ,
    ros_types::PointCloud2Msg,
    ConvertXYZ,
};

// Your points (here using the predefined type PointXYZ).
let cloud_points = vec![
  PointXYZ {
    x: 1.3,
    y: 1.6,
    z: 5.7,
  },
  PointXYZ {
    x: f32::MAX,
    y: f32::MIN,
    z: f32::MAX,
  },
];

let cloud_copy = cloud_points.clone(); // For checking equality later.

// Vector -> Converter -> Message
let internal_msg: PointCloud2Msg = ConvertXYZ::try_from(cloud_points)
    .unwrap()
    .try_into()
    .unwrap();

// Convert to your favorite ROS crate message type, we will use rosrust here.
// let msg: rosrust_msg::sensor_msgs::PointCloud2 = internal_msg.into();

// Back to this crate's message type.
// let internal_msg: PointCloud2Msg = msg.into();

// Message -> Converter -> Vector
let convert: ConvertXYZ = ConvertXYZ::try_from(internal_msg).unwrap();
let new_cloud_points = convert
    .map(|point: PointXYZ| {
      // Insert your point business logic here
      // or use other methods like .for_each().
      
      // We are using a fallible iterator so we need to return a Result.
      Ok(point)
    })
    .collect::<Vec<PointXYZ>>()
    .unwrap(); // Handle point conversion or byte errors here.

assert_eq!(new_cloud_points, cloud_copy);
```

To use `ros_pointcloud2` in your favorite ROS crate, you can either use this crate's features (see Integration section below) or implement the `Into` and `From` traits for `PointCloud2Msg`.

Try to avoid cloning the `data: Vec<u8>` field.
```rust
use ros_pointcloud2::ros_types::PointCloud2Msg;

struct YourROSPointCloud2 {} // Likely to be generated by your ROS crate.

impl Into<YourROSPointCloud2> for PointCloud2Msg {
  fn into(self) -> YourROSPointCloud2 {
    todo!()
  }
}

impl From<YourROSPointCloud2> for PointCloud2Msg {
  fn from(msg: YourROSPointCloud2) -> Self {
    todo!()
  }
}
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

You can use them by enabling the corresponding feature. Example:
```toml
[dependencies]
ros_pointcloud2 = { version = "*", features = ["r2r_msg"]}
```

For `rclrs_msg`, features do not work properly because of the way it generates the messages. You need to use tags instead:
```toml
[dependencies]
ros_pointcloud2 = { git = "https://github.com/stelzo/ros_pointcloud2", tag = "v0.3.2_rclrs", features = ["rclrs_msg"] }
```

Please open an issue or PR if you want to see support for other crates.

## Features

- Easy to integrate into your favorite ROS1 or ROS2 crate
- Custom point types
- Predefined types for common conversions (compared to PCL)
  - PointXYZ
  - PointXYZI
  - PointXYZL
  - PointXYZRGB
  - PointXYZRGBA
  - PointXYZRGBL
  - PointXYZNormal
  - PointXYZINormal
  - PointXYZRGBNormal
- 2D and 3D

## Custom Types

You can freely convert to your own point types without reiterating the entire cloud.

You just need to implement some traits.
```rust
use ros_pointcloud2::mem_macros::size_of;
use ros_pointcloud2::ros_types::PointCloud2Msg;
use ros_pointcloud2::{ConversionError, Convert, MetaNames, PointConvertible, PointMeta};

const DIM: usize = 3;
const METADIM: usize = 4;

#[derive(Debug, PartialEq, Clone)]
struct CustomPoint {
  x: f32, // DIM 1
  y: f32, // DIM 2
  z: f32, // DIM 3
  r: u8,  // METADIM 1
  g: u8,  // METADIM 2
  b: u8,  // METADIM 3
  a: u8,  // METADIM 4
}

// Converting your custom point to the crate's internal representation
impl From<CustomPoint> for ([f32; DIM], [PointMeta; METADIM]) {
  fn from(point: CustomPoint) -> Self {
    (
      [point.x, point.y, point.z],
      [
        PointMeta::new(point.r),
        PointMeta::new(point.g),
        PointMeta::new(point.b),
        PointMeta::new(point.a),
      ],
    )
  }
}

// The mappings for index of meta idx to field names. Example: 0 -> "r", 1 -> "g", 2 -> "b", 3 -> "a"
impl MetaNames<METADIM> for CustomPoint {
  fn meta_names() -> [String; METADIM] {
    ["r", "g", "b", "a"].map(|s| s.to_string())
  }
}

// Converting crate's internal representation to your custom point
impl TryFrom<([f32; DIM], [PointMeta; METADIM])> for CustomPoint {
  type Error = ConversionError;
  fn try_from(data: ([f32; DIM], [PointMeta; METADIM])) -> Result<Self, Self::Error> {
    Ok(Self {
      x: data.0[0],
      y: data.0[1],
      z: data.0[2],
      r: data.1[0].get()?,
      g: data.1[1].get()?,
      b: data.1[2].get()?,
      a: data.1[3].get()?,
    })
  }
}

impl PointConvertible<f32, { size_of!(f32) }, DIM, METADIM> for CustomPoint {}

type MyConverter = Convert<f32, { size_of!(f32) }, DIM, METADIM, CustomPoint>;

// Your custom cloud (Vector)
let custom_cloud = vec![
  CustomPoint {
    x: f32::MAX,
    y: f32::MIN,
    z: f32::MAX,
    r: u8::MAX,
    g: u8::MAX,
    b: u8::MAX,
    a: u8::MAX,
  },
];

// Cloud -> ROS message
let custom_msg: PointCloud2Msg = MyConverter::try_from(custom_cloud)
    .unwrap()
    .try_into()
    .unwrap();

// ROS message -> Cloud
let to_custom_type = MyConverter::try_from(custom_msg).unwrap();
```

## Future Work
- Benchmark vs PCL
- Proper error passing to the iterator result (currently merged into `PointConversionError`)
- remove allocations
- introduce no-panic for maximum stability
- Add more predefined types

## License
[MIT](https://choosealicense.com/licenses/mit/)
