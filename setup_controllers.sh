#!/usr/bin/env bash
set -e

SRC_DIR="ros2_ws/src"
echo "🔧 Setting up CMakeLists.txt for controllers in $SRC_DIR..."

# Loop through *_controller packages
for dir in "$SRC_DIR"/*_controller; do
  [ -d "$dir" ] || continue
  pkg_name=$(basename "$dir")
  script_name="${pkg_name%.sh}"  # Strip .sh if misnamed

  # Try to auto-detect main Python script
  py_file=$(find "$dir" -name "*.py" | head -n 1)
  if [[ -z "$py_file" ]]; then
    echo "⚠️  No Python file found in $pkg_name. Skipping."
    continue
  fi

  rel_path="${py_file##*/}"
  echo "📦 Generating CMakeLists.txt for $pkg_name (script: $rel_path)..."

  cat <<EOF > "$dir/CMakeLists.txt"
cmake_minimum_required(VERSION 3.5)
project($pkg_name)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  PROGRAMS
    $pkg_name/$rel_path
  DESTINATION lib/\${PROJECT_NAME}
)

ament_package()
EOF
done

echo "✅ Done. Now rebuild with:"
echo "  cd ros2_ws && rm -rf build install log && source /opt/ros/humble/setup.bash && colcon build"
