// This example demonstrates how to use custom point types with custom metadata.
// The use case is a segmentation point cloud where each point holds a label
// with a custom enum type we want to filter.

use ros_pointcloud2::{Fields, Point};

#[cfg(not(feature = "derive"))]
use ros_pointcloud2::{PointCloud2Msg, PointConvertible};

#[derive(Debug, PartialEq, Clone, Default)]
enum Label {
    #[default]
    Human,
    Deer,
    Car,
}

impl From<Label> for u8 {
    fn from(label: Label) -> Self {
        match label {
            Label::Human => 0,
            Label::Deer => 1,
            Label::Car => 2,
        }
    }
}

impl From<u8> for Label {
    fn from(label: u8) -> Self {
        match label {
            0 => Label::Human,
            1 => Label::Deer,
            2 => Label::Car,
            _ => panic!("Invalid label"),
        }
    }
}

// Custom point type with custom metadata
#[derive(Debug, PartialEq, Clone, Default)]
struct CustomPoint {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32,
    my_custom_label: Label,
}

// We implement the PointConvertible trait

// A ros_pointcloud2::Point takes as generic arguments:
// - coordinate type
// - dimension (xyz = 3)
// - metadata dimension (intensity + my_custom_label = 2)
impl From<CustomPoint> for Point<5> {
    fn from(point: CustomPoint) -> Self {
        Point {
            fields: [
                point.x.into(),
                point.y.into(),
                point.z.into(),
                point.intensity.into(),
                u8::from(point.my_custom_label).into(),
            ],
        }
    }
}

impl From<Point<5>> for CustomPoint {
    fn from(point: Point<5>) -> Self {
        let label_raw: u8 = point.fields[1].get();
        Self {
            x: point.fields[0].get(),
            y: point.fields[1].get(),
            z: point.fields[2].get(),
            intensity: point.fields[3].get(),
            my_custom_label: Label::from(label_raw), // decoding the label from u8
        }
    }
}

impl Fields<5> for CustomPoint {
    fn field_names_ordered() -> [&'static str; 5] {
        ["x", "y", "z", "intensity", "my_custom_label"] // how we want to name the metadata in the message
    }
}

// We implemented everything that is needed so we declare it as a PointConvertible
#[cfg(not(feature = "derive"))]
impl PointConvertible<5> for CustomPoint {}

fn main() {
    #[cfg(not(feature = "derive"))]
    {
        let cloud = vec![
            CustomPoint {
                x: 1.0,
                y: 2.0,
                z: 3.0,
                intensity: 4.0,
                my_custom_label: Label::Deer,
            },
            CustomPoint {
                x: 4.0,
                y: 5.0,
                z: 6.0,
                intensity: 7.0,
                my_custom_label: Label::Car,
            },
            CustomPoint {
                x: 7.0,
                y: 8.0,
                z: 9.0,
                intensity: 10.0,
                my_custom_label: Label::Human,
            },
        ];

        println!("Original cloud: {:?}", cloud);

        let msg = PointCloud2Msg::try_from_iter(cloud.clone().into_iter()).unwrap();

        println!("filtering by label == Deer");
        let out = msg
            .try_into_iter()
            .unwrap()
            .filter(|point: &CustomPoint| point.my_custom_label == Label::Deer)
            .collect::<Vec<_>>();

        println!("Filtered cloud: {:?}", out);

        assert_eq!(
            vec![CustomPoint {
                x: 1.0,
                y: 2.0,
                z: 3.0,
                intensity: 4.0,
                my_custom_label: Label::Deer,
            },],
            out
        );
    }
}
