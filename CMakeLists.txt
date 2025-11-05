cmake_minimum_required(VERSION 3.15)
project(kdl_parser)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdfdom_headers REQUIRED)
# 一些发行版把 urdfdom 的 CMake 包名叫 urdfdom；若找不到，可以直接用 include dirs + lib
find_package(urdfdom QUIET)

add_library(kdl_parser
  src/kdl_parser.cpp
)
# 使用 PRIVATE 确保构建时能找到头文件，PUBLIC 确保使用该库的其他目标也能找到
target_include_directories(kdl_parser
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${urdfdom_headers_INCLUDE_DIRS}
)

# 如果找到了 urdfdom 包，添加其包含路径
if(urdfdom_FOUND)
  target_include_directories(kdl_parser PRIVATE ${urdfdom_INCLUDE_DIRS})
endif()
target_link_libraries(kdl_parser PUBLIC
  orocos-kdl
  Eigen3::Eigen
)
# urdfdom 提供的库名称在不同平台可能不同，常见是 urdfdom_model、urdfdom_sensor 等
# 这里主要用到 model & parser；根据你安装的包名进行调整：
find_library(URDFDOM_MODEL_LIB urdfdom_model)
find_library(URDFDOM_PARSER_LIB urdfdom_world) # 或 urdfdom, urdfdom_model_state，不同发行版不同
# 若找不到，改为：
# find_library(URDFDOM_PARSER_LIB urdfdom)
if(URDFDOM_MODEL_LIB)
  target_link_libraries(kdl_parser PUBLIC ${URDFDOM_MODEL_LIB})
endif()
if(URDFDOM_PARSER_LIB)
  target_link_libraries(kdl_parser PUBLIC ${URDFDOM_PARSER_LIB})
endif()

# Windows 下：默认构建静态库，不需要导出符号
# 如果需要构建 DLL，可以取消注释下面这行，并恢复 visibility_control.hpp 的使用
# if(WIN32)
#   target_compile_definitions(kdl_parser PRIVATE "KDL_PARSER_BUILDING_DLL")
# endif()

# 添加测试程序（可选）
option(BUILD_TEST "Build test program" ON)
if(BUILD_TEST AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test_so100.cpp)
  add_executable(test_so100 test_so100.cpp)
  # 确保使用项目本地的头文件，而不是 vcpkg 或其他地方的旧版本
  # 将项目包含路径放在最前面，确保优先级最高
  target_include_directories(test_so100 BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_so100 kdl_parser)
  # 设置工作目录，使程序能找到 URDF 文件
  set_target_properties(test_so100 PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()
