use ros_pointcloud2::PointConvertible;
use rpcl2_derive::*;

#[derive(Debug, PartialEq, Clone, Default, PointConvertible)]
#[repr(C, align(4))]
struct MyPointXYZI {
    x: f32,
    #[rpcl2(rename("test"))]
    y: u16,
    z: f32,
    #[rpcl2(rename("i"))]
    intensity: i32,
    label: u8,
}

#[test]
fn layout() {
    let layout_str = format!("{:?}", MyPointXYZI::layout());
    assert_eq!("LayoutDescription([Field { name: \"x\", ty: \"f32\", size: 4 }, Field { name: \"test\", ty: \"u16\", size: 2 }, Padding { size: 2 }, Field { name: \"z\", ty: \"f32\", size: 4 }, Field { name: \"i\", ty: \"i32\", size: 4 }, Field { name: \"label\", ty: \"u8\", size: 1 }, Padding { size: 3 }])", layout_str);
}
