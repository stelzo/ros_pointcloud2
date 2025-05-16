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

// Define a custom point with an enum.
// This is normally not supported by PointCloud2 but we will explain the library how to handle it.
#[derive(Debug, PartialEq, Clone, Default, Copy)]
#[repr(C, align(4))]
struct CustomPoint {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32,
    my_custom_label: Label,
}

// Some convenience functions to convert between the enum and u8.
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

impl CustomPoint {
    fn new(x: f32, y: f32, z: f32, intensity: f32, my_custom_label: Label) -> Self {
        Self {
            x,
            y,
            z,
            intensity,
            my_custom_label,
        }
    }
}

// We implement the PointConvertible trait (needed for every custom point).
// IPoint is the internal representation. It takes the amount of fields as generic arguments.
impl From<CustomPoint> for IPoint<5> {
    fn from(point: CustomPoint) -> Self {
        [
            point.x.into(),
            point.y.into(),
            point.z.into(),
            point.intensity.into(),
            u8::from(point.my_custom_label).into(),
        ]
        .into()
    }
}

impl From<IPoint<5>> for CustomPoint {
    fn from(point: IPoint<5>) -> Self {
        Self::new(
            point[0].get(),
            point[1].get(),
            point[2].get(),
            point[3].get(),
            point[4].get(),
        )
    }
}

// C representation of the struct hardcoded without using the derive feature.
unsafe impl PointConvertible<5> for CustomPoint {
    fn layout() -> LayoutDescription {
        LayoutDescription::new(&[
            LayoutField::new("x", "f32", 4),
            LayoutField::new("y", "f32", 4),
            LayoutField::new("z", "f32", 4),
            LayoutField::new("intensity", "f32", 4),
            LayoutField::new("my_custom_label", "u8", 1),
            LayoutField::padding(3),
        ])
    }
}

// Now we tell the library how to encode and decode the label.
// You don't need to do this if your CustomPoint has a field that is already supported by PointCloud2.
impl GetFieldDatatype for Label {
    fn field_datatype() -> FieldDatatype {
        FieldDatatype::U8 // Declare that we want to use u8 as the datatype for the label.
    }
}

// Again, you don't need this with only supported field types.
// u8 -> Label
impl FromBytes for Label {
    // Technically, PointCloud2 supports big and little endian even though it is rarely used.
    // 'be' stands for big endian and 'le' for little endian.
    fn from_be_bytes(bytes: PointDataBuffer) -> Self {
        u8::from_be_bytes([bytes[0]]).into()
    }

    fn from_le_bytes(bytes: PointDataBuffer) -> Self {
        u8::from_le_bytes([bytes[0]]).into()
    }
}

// Label -> u8
impl From<Label> for PointDataBuffer {
    fn from(label: Label) -> Self {
        [u8::from(label)].into()
    }
}

fn main() {
    let cloud = vec![
        CustomPoint::new(1.0, 2.0, 3.0, 4.0, Label::Deer),
        CustomPoint::new(4.0, 5.0, 6.0, 7.0, Label::Car),
        CustomPoint::new(7.0, 8.0, 9.0, 10.0, Label::Human),
    ];

    println!("Original cloud: {cloud:?}");

    let msg = PointCloud2Msg::try_from_iter(&cloud).unwrap();

    println!("filtering by label == Deer");
    let out = msg
        .try_into_iter()
        .unwrap()
        .filter(|point: &CustomPoint| point.my_custom_label == Label::Deer)
        .collect::<Vec<_>>();

    println!("Filtered cloud: {out:?}");

    assert_eq!(
        vec![CustomPoint::new(1.0, 2.0, 3.0, 4.0, Label::Deer),],
        out
    );
}
