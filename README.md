# ROS PointCloud2

A Rust implementation for fast, safe and customizable conversions to and from the `sensor_msgs/PointCloud2` ROS message.

Providing a fast and memory efficient way for message conversion while allowing user defined types without the cost of iterations.

*Currently only supports [rosrust_msg](https:://github.com/adnanademovic/rosrust) with ROS1.*

```rust
use ros_pointcloud2::ConvertXYZ;
use ros_pointcloud2::pcl_utils::PointXYZ;
use ros_pointcloud2::fallible_iterator::FallibleIterator;

use rosrust_msg::sensor_msgs::PointCloud2;

// Your points (here using the predefined type PointXYZ).
let cloud_points = vec![
  PointXYZ { x: 1.3, y: 1.6, z: 5.7 },
  PointXYZ { x: f32::MAX, y: f32::MIN, z: f32::MAX },
];

let cloud_copy = cloud_points.clone(); // Only for checking equality later.

// Vector -> Convert -> Message
let msg: PointCloud2 = ConvertXYZ::try_from(cloud_points).unwrap().try_into().unwrap();

// Message -> Convert -> Vector
let convert: ConvertXYZ = ConvertXYZ::try_from(msg).unwrap();
let new_cloud_points = convert.map(|point: PointXYZ| {
    // Insert your point business logic here or use other methods like .for_each().
    // I will just copy the points into a vector as an example.
    // Also, since we are using a fallible iterator, we need to return a Result.
    Ok(point)
}).collect::<Vec<PointXYZ>>()
  .unwrap(); // Handle point conversion or byte errors here.

assert_eq!(new_cloud_points, cloud_copy);
```

Instead of converting the entire cloud into a `Vector`, you get an `Iterator` that converts each point from the message on the fly.
An example for using this crate is [this filter node](https://github.com/stelzo/cloudfilter). It is also a good starting point for
implementing ROS1 nodes in Rust inside a catkin environment.

## Features

- Full support for `sensor_msgs/PointCloud2` messages
- Custom types with `From` and `Into` traits
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
- 2D and 3D support

## Usage

Add this to your Cargo.toml:
```toml
[dependencies]
ros_pointcloud2 = "0.1.0"
```

When building, the `rosrust_msg` crate needs to have the ROS environment sourced or use `ROSRUST_MSG_PATH=/opt/ros/$ROS_DISTRO/share cargo build`.

## Custom Types

You can freely convert to your own types without reiterating the entire cloud.

You just need to implement the `Into` and `From` traits.
```rust
use ros_pointcloud2::mem_macros::size_of;
use ros_pointcloud2::{Convert, MetaNames, PointMeta, ConversionError, PointConvertible};
use rosrust_msg::sensor_msgs::PointCloud2;

const DIM : usize = 3;
const METADIM : usize = 4;

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
impl Into<([f32; DIM], [PointMeta; METADIM])> for CustomPoint {
  fn into(self) -> ([f32; DIM], [PointMeta; METADIM]) {
    ([self.x, self.y, self.z], [PointMeta::new(self.r), PointMeta::new(self.g), PointMeta::new(self.b), PointMeta::new(self.a)])
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
  CustomPoint { x: 0.0, y: 1.0, z: 5.0, r: 0, g: 0, b: 0, a: 0 },
  CustomPoint { x: 1.0, y: 1.5, z: 5.0, r: 1, g: 1, b: 1, a: 1 },
  CustomPoint { x: 1.3, y: 1.6, z: 5.7, r: 2, g: 2, b: 2, a: 2 },
  CustomPoint { x: f32::MAX, y: f32::MIN, z: f32::MAX, r: u8::MAX, g: u8::MAX, b: u8::MAX, a: u8::MAX },
];


// Cloud -> ROS message
let custom_msg: PointCloud2 = MyConverter::try_from(custom_cloud).unwrap().try_into().unwrap();

// ROS message -> Cloud
let to_custom_type = MyConverter::try_from(custom_msg).unwrap();
```

## Future Work
- ROS2 support(!)
- Removing rosrust dependency
- Benchmark vs PCL
- Proper error passing to the iterator result (currently merged into `PointConversionError`)
- remove allocations
- introduce no-panic for maximum stability
- Add more predefined types

## License
[MIT](https://choosealicense.com/licenses/mit/)
