cmake_minimum_required(VERSION 3.16)
project(zeus_planning VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ─── boost_plugin_loader (헤더만 사용, CMakeLists.txt 실행 안 함) ──────────────
# noether_tpp 헤더가 boost_plugin_loader 타입을 참조하므로 헤더만 필요.
# 실제 플러그인 로딩 기능은 사용하지 않음.
FetchContent_Declare(
  boost_plugin_loader
  GIT_REPOSITORY https://github.com/tesseract-robotics/boost_plugin_loader.git
  GIT_TAG        0.4.3
)
FetchContent_Populate(boost_plugin_loader)
FetchContent_GetProperties(boost_plugin_loader SOURCE_DIR BPL_SRC)

add_library(boost_plugin_loader_iface INTERFACE)
add_library(boost_plugin_loader::boost_plugin_loader ALIAS boost_plugin_loader_iface)
target_include_directories(boost_plugin_loader_iface INTERFACE ${BPL_SRC}/include)
target_link_libraries(boost_plugin_loader_iface INTERFACE
  Boost::filesystem
  Boost::system
  ${CMAKE_DL_LIBS}
)

# ─── System Dependencies ───────────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system program_options)
find_package(PCL REQUIRED COMPONENTS common io features kdtree surface segmentation)
find_package(yaml-cpp REQUIRED)
find_package(VTK 9 REQUIRED COMPONENTS
  CommonCore
  CommonDataModel
  CommonTransforms
  CommonColor
  CommonMath
  CommonComputationalGeometry
  IOGeometry
  IOPLY
  FiltersCore
  FiltersGeneral
  RenderingCore
  RenderingAnnotation
  RenderingOpenGL2
)

if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen IMPORTED INTERFACE)
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

# ─── noether_tpp (plugin system 제외, ROS 의존성 없음) ────────────────────────
set(NOETHER_TPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/noether/noether_tpp)

# PCL NURBS(B-Spline) 지원 여부 확인
try_compile(
  NURBS_FOUND ${CMAKE_CURRENT_BINARY_DIR}
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/test/check_pcl_nurbs.cpp
  CMAKE_FLAGS -DINCLUDE_DIRECTORIES=${PCL_INCLUDE_DIRS}
  LINK_LIBRARIES pcl_surface
)
if(NURBS_FOUND)
  message(STATUS "PCL NURBS found: BSpline modifier enabled")
else()
  message(STATUS "PCL NURBS not found: BSpline modifier disabled")
endif()

set(NOETHER_TPP_SOURCES
  # Core
  ${NOETHER_TPP_DIR}/src/core/tool_path_planner_pipeline.cpp
  ${NOETHER_TPP_DIR}/src/splines.cpp
  ${NOETHER_TPP_DIR}/src/utils.cpp
  # Mesh Modifiers
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/subset_extraction/subset_extractor.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/subset_extraction/extruded_polygon_subset_extractor.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/compound_modifier.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/clean_data_modifier.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/euclidean_clustering_modifier.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/face_subdivision_modifier.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/fill_holes_modifier.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/windowed_sinc_smoothing.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/ransac_primitive_fit_modifier.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/ransac_cylinder_fit_modifier.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/ransac_plane_fit_modifier.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/normal_estimation_pcl.cpp
  ${NOETHER_TPP_DIR}/src/mesh_modifiers/normals_from_mesh_faces_modifier.cpp
  # Tool Path Modifiers
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/biased_tool_drag_orientation_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/circular_lead_in_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/circular_lead_out_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/compound_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/concatenate_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/direction_of_travel_orientation_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/fixed_orientation_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/moving_average_orientation_smoothing_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/raster_organization_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/snake_organization_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/standard_edge_paths_organization_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/join_close_segments_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/linear_approach_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/linear_departure_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/minimum_segment_length_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/spline_extrapolation_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/tool_drag_orientation_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/uniform_orientation_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/offset_modifier.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_modifiers/uniform_spacing_modifier.cpp
  # Tool Path Planners
  ${NOETHER_TPP_DIR}/src/tool_path_planners/multi_tool_path_planner.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/edge/edge_planner.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/edge/boundary_edge_planner.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/raster/raster_planner.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/raster/origin_generators/aabb_center_origin_generator.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/raster/origin_generators/centroid_origin_generator.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/raster/origin_generators/offset_origin_generator.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/raster/origin_generators/fixed_origin_generator.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/raster/direction_generators/fixed_direction_generator.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/raster/direction_generators/pca_rotated_direction_generator.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/raster/plane_slicer_raster_planner.cpp
  ${NOETHER_TPP_DIR}/src/tool_path_planners/raster/plane_slicer_legacy_raster_planner.cpp
)

if(NURBS_FOUND)
  list(APPEND NOETHER_TPP_SOURCES
    ${NOETHER_TPP_DIR}/src/mesh_modifiers/bspline_reconstruction_modifier.cpp
  )
endif()

add_library(noether_tpp SHARED ${NOETHER_TPP_SOURCES})

target_compile_definitions(noether_tpp PUBLIC
  NOETHER_TPP_SECTION=tpp
  NOETHER_DIRECTION_GENERATOR_SECTION=dg
  NOETHER_ORIGIN_GENERATOR_SECTION=og
  NOETHER_TOOL_PATH_MODIFIER_SECTION=mod
  NOETHER_MESH_MODIFIER_SECTION=mesh
)

target_include_directories(noether_tpp
  PUBLIC  ${NOETHER_TPP_DIR}/include
  SYSTEM PUBLIC ${PCL_INCLUDE_DIRS}
)

target_link_libraries(noether_tpp
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    yaml-cpp
    boost_plugin_loader::boost_plugin_loader
)

vtk_module_autoinit(
  TARGETS noether_tpp
  MODULES ${VTK_LIBRARIES}
)

# ─── Planning Executable ──────────────────────────────────────────────────────
add_executable(planning src/main.cpp src/factory_stub.cpp)
target_link_libraries(planning PRIVATE noether_tpp Boost::program_options ${PCL_LIBRARIES})
