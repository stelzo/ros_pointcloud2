fn main() -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(feature = "roslibrust")]
    {
        let (source_ros1, dependent_paths_ros1) =
            roslibrust::codegen::find_and_generate_ros_messages_without_ros_package_path(vec![
                "./assets/ros1".into(),
            ])?;
        let (source_ros2, dependent_paths_ros2) =
            roslibrust::codegen::find_and_generate_ros_messages_without_ros_package_path(vec![
                "./assets/ros2_common_interfaces".into(),
                "./assets/ros2_required_msgs/rcl_interfaces/builtin_interfaces".into(),
            ])?;
        let out_dir = std::env::var_os("OUT_DIR").unwrap();
        let dest_path_ros1 = std::path::Path::new(&out_dir).join("ros1_messages.rs");
        std::fs::write(dest_path_ros1, source_ros1.to_string())?;
        let dest_path_ros2 = std::path::Path::new(&out_dir).join("ros2_messages.rs");
        std::fs::write(dest_path_ros2, source_ros2.to_string())?;

        for path in &dependent_paths_ros1 {
            println!("cargo:rerun-if-changed={}", path.display());
        }

        for path in &dependent_paths_ros2 {
            println!("cargo:rerun-if-changed={}", path.display());
        }
    }

    Ok(())
}
