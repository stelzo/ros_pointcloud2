/// This example demonstrates how to use a custom point with encoded metadata.
/// The use case is a segmentation point cloud where each point holds a label and we want to filter by it.
/// Since the datatypes for the PointCloud2 message are very limited,
/// we need to encode the enum into a supported type.
/// This needs some manual work to tell the library how to encode and decode the enum.
///
/// Important Note: This example is only possible with disabled `derive` feature,
/// because the library (currently) does not know the size of your chosen supported type at compile time.
/// This makes direct copies impossible.
use ros_pointcloud2::prelude::*;

#[derive(Debug, PartialEq, Clone, Default, Copy)]
enum Label {
    #[default]
    Human,
    Deer,
    Car,
}

// Define a custom point with an enum (not supported by PointCloud2)
#[derive(Debug, PartialEq, Clone, Default)]
struct CustomPoint {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32,
    my_custom_label: Label,
}

// For our convenience
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

// We implement the PointConvertible trait (needed for every custom point)

// A ros_pointcloud2::Point takes as generic arguments:
// - coordinate type
// - dimension (xyz = 3)
// - metadata dimension (intensity + my_custom_label = 2)
impl From<CustomPoint> for RPCL2Point<5> {
    fn from(point: CustomPoint) -> Self {
        RPCL2Point {
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

impl From<RPCL2Point<5>> for CustomPoint {
    fn from(point: RPCL2Point<5>) -> Self {
        Self {
            x: point.fields[0].get(),
            y: point.fields[1].get(),
            z: point.fields[2].get(),
            intensity: point.fields[3].get(),
            my_custom_label: point.fields[4].get(), // decoding the label from u8
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

// Now we tell the library how to encode and decode the label.
// You don't need to do this if your CustomPoint has a field that is already supported by PointCloud2.
impl GetFieldDatatype for Label {
    fn field_datatype() -> FieldDatatype {
        FieldDatatype::U8 // we want to encode the label as u8
    }
}

// Again, you don't need this with only supported field types.
// Technically, PointCloud2 supports big and little endian even though it is rarely used.
// 'be' stands for big endian and 'le' for little endian.
impl FromBytes for Label {
    // u8 -> Label
    fn from_be_bytes(bytes: &[u8]) -> Self {
        u8::from_be_bytes([bytes[0]]).into()
    }

    fn from_le_bytes(bytes: &[u8]) -> Self {
        u8::from_le_bytes([bytes[0]]).into()
    }

    // Label -> u8
    fn bytes(label: &Self) -> Vec<u8> {
        u8::bytes(&u8::from(*label))
    }
}

fn main() {
    #[cfg(not(feature = "derive"))]
    {
        let cloud = vec![
            CustomPoint::new(1.0, 2.0, 3.0, 4.0, Label::Deer),
            CustomPoint::new(4.0, 5.0, 6.0, 7.0, Label::Car),
            CustomPoint::new(7.0, 8.0, 9.0, 10.0, Label::Human),
        ];

        println!("Original cloud: {:?}", cloud);

        let msg = PointCloud2Msg::try_from_iter(cloud).unwrap();

        println!("filtering by label == Deer");
        let out = msg
            .try_into_iter()
            .unwrap()
            .filter(|point: &CustomPoint| point.my_custom_label == Label::Deer)
            .collect::<Vec<_>>();

        println!("Filtered cloud: {:?}", out);

        assert_eq!(
            vec![CustomPoint::new(1.0, 2.0, 3.0, 4.0, Label::Deer),],
            out
        );
    }
}
