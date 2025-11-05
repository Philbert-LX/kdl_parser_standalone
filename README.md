# KDL Parser (Standalone)

A standalone C++ library for converting URDF (Unified Robot Description Format) files to KDL (Kinematics and Dynamics Library) tree structures. This is a **ROS-independent** version that can be compiled and used on Windows, Linux, and macOS without any ROS dependencies.

## Features

- ✅ **ROS-free**: Completely independent from ROS/ROS2, no `ament_cmake`, `rcutils`, or other ROS dependencies
- ✅ **Cross-platform**: Supports Windows, Linux, and macOS
- ✅ **Pure CMake**: Uses standard CMake build system (no ROS build tools required)
- ✅ **C++17**: Modern C++ standard support
- ✅ **Lightweight**: Minimal dependencies (only KDL, urdfdom, and Eigen3)

## Requirements

- **CMake** >= 3.15
- **C++17** compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- **Dependencies**:
  - [orocos-kdl](https://github.com/orocos/orocos_kinematics_dynamics) - Kinematics and Dynamics Library
  - [urdfdom](https://github.com/ros/urdfdom) - URDF parser library
  - [Eigen3](https://eigen.tuxfamily.org/) - Linear algebra library

## Installation

### Windows (using vcpkg)

1. Install [vcpkg](https://github.com/microsoft/vcpkg) if you haven't already:
   ```powershell
   git clone https://github.com/microsoft/vcpkg.git
   cd vcpkg
   .\bootstrap-vcpkg.bat
   ```

2. Install dependencies:
   ```powershell
   .\vcpkg install orocos-kdl:x64-windows
   .\vcpkg install urdfdom:x64-windows
   .\vcpkg install eigen3:x64-windows
   ```

3. Configure CMake with vcpkg toolchain:
   ```powershell
   cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=[path-to-vcpkg]/scripts/buildsystems/vcpkg.cmake
   ```

### Linux (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install libeigen3-dev liborocos-kdl-dev liburdfdom-dev
```

### macOS (using Homebrew)

```bash
brew install eigen orocos-kdl urdfdom
```

## Building

### Standard Build

```bash
# Configure
cmake -B build -S .

# Build
cmake --build build

# (Optional) Install
cmake --install build --prefix install
```

### Build with Tests

The project includes a test program that can be enabled:

```bash
cmake -B build -S . -DBUILD_TEST=ON
cmake --build build
```

### Windows (Visual Studio)

```powershell
# Generate Visual Studio solution
cmake -B build -S . -G "Visual Studio 17 2022" -A x64

# Open kdl_parser.sln in Visual Studio or build from command line
cmake --build build --config Release
```

## Usage

### Basic Example

```cpp
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/tree.hpp"

int main() {
    KDL::Tree tree;
    
    // Parse URDF from file
    if (!kdl_parser::treeFromFile("robot.urdf", tree)) {
        std::cerr << "Failed to parse URDF file" << std::endl;
        return 1;
    }
    
    // Use the KDL tree for kinematics calculations
    // ...
    
    return 0;
}
```

### Parse from String

```cpp
std::string urdf_string = "<?xml version=\"1.0\"?>...";
KDL::Tree tree;
if (kdl_parser::treeFromString(urdf_string, tree)) {
    // Success
}
```

### Parse from URDF Model

```cpp
#include "urdf_model/model.h"
#include "urdf_parser/urdf_parser.h"

urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_string);
KDL::Tree tree;
if (kdl_parser::treeFromUrdfModel(*model, tree)) {
    // Success
}
```

## API Reference

The library provides three main functions in the `kdl_parser` namespace:

- `bool treeFromFile(const std::string &file, KDL::Tree &tree)` - Parse URDF from a file
- `bool treeFromString(const std::string &xml, KDL::Tree &tree)` - Parse URDF from a string
- `bool treeFromUrdfModel(const urdf::ModelInterface &robot_model, KDL::Tree &tree)` - Convert from URDF model object

All functions return `true` on success and `false` on failure.

## Integration with Your Project

### Using CMake

```cmake
# Find the library
find_package(kdl_parser REQUIRED)

# Link to your target
target_link_libraries(your_target PRIVATE kdl_parser)
```

### Manual Integration

1. Add `include/` to your include paths
2. Link against the `kdl_parser` library
3. Ensure KDL, urdfdom, and Eigen3 are available

## Differences from ROS Version

This standalone version differs from the original ROS `kdl_parser` package in the following ways:

- ❌ No ROS/ROS2 dependencies (`ament_cmake`, `rcutils`, etc.)
- ❌ No ROS-specific logging (uses standard C++ `fprintf` instead)
- ✅ Can be compiled independently on Windows without ROS
- ✅ Uses standard CMake instead of `ament_cmake`
- ✅ No `package.xml` required

The API is **fully compatible** with the ROS version, so existing code using `kdl_parser` should work without modification.

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

This project is based on the original [kdl_parser](https://github.com/ros/kdl_parser) by Willow Garage, with all ROS dependencies removed to enable standalone usage.
