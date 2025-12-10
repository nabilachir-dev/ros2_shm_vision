cmake_minimum_required(VERSION 3.8)
project(ros2_shm_vision)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# DEPENDENCIES / DIPENDENZE
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ============================================================================
# MESSAGE GENERATION / GENERAZIONE MESSAGGI
# ============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Image1080p.msg"
)

# Get typesupport target for linking
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ============================================================================
# EXECUTABLES / ESEGUIBILI
# ============================================================================

# --- SIM Library (Sensor-in-Memory) ---
add_library(sim_transport SHARED src/sim_transport.cpp)
target_compile_features(sim_transport PUBLIC cxx_std_17)
target_include_directories(sim_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(sim_transport rt pthread)

# --- SIM Publisher (with fallback) ---
add_executable(sim_publisher_1080p src/sim_publisher_1080p.cpp)
target_compile_features(sim_publisher_1080p PUBLIC cxx_std_17)
ament_target_dependencies(sim_publisher_1080p "rclcpp" "std_msgs" "sensor_msgs")
target_link_libraries(sim_publisher_1080p "${cpp_typesupport_target}" ${OpenCV_LIBS} sim_transport)
target_include_directories(sim_publisher_1080p PUBLIC 
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# --- SIM Subscriber (with fallback) ---
add_executable(sim_subscriber_1080p src/sim_subscriber_1080p.cpp)
target_compile_features(sim_subscriber_1080p PUBLIC cxx_std_17)
ament_target_dependencies(sim_subscriber_1080p "rclcpp" "std_msgs" "sensor_msgs")
target_link_libraries(sim_subscriber_1080p "${cpp_typesupport_target}" ${OpenCV_LIBS} sim_transport)
target_include_directories(sim_subscriber_1080p PUBLIC 
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# --- SIM Pure Publisher (no fallback, no checksum) ---
add_executable(sim_publisher_nofb_1080p src/sim_publisher_nofb_1080p.cpp)
target_compile_features(sim_publisher_nofb_1080p PUBLIC cxx_std_17)
ament_target_dependencies(sim_publisher_nofb_1080p "rclcpp")
target_link_libraries(sim_publisher_nofb_1080p ${OpenCV_LIBS} sim_transport)
target_include_directories(sim_publisher_nofb_1080p PUBLIC 
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# --- SIM Pure Subscriber (no fallback, no checksum) ---
add_executable(sim_subscriber_nofb_1080p src/sim_subscriber_nofb_1080p.cpp)
target_compile_features(sim_subscriber_nofb_1080p PUBLIC cxx_std_17)
ament_target_dependencies(sim_subscriber_nofb_1080p "rclcpp")
target_link_libraries(sim_subscriber_nofb_1080p ${OpenCV_LIBS} sim_transport)
target_include_directories(sim_subscriber_nofb_1080p PUBLIC 
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)


# --- Loaned Publisher (Zero-Copy) ---
add_executable(loaned_publisher_1080p src/loaned_publisher_1080p.cpp)
target_compile_features(loaned_publisher_1080p PUBLIC cxx_std_17)
ament_target_dependencies(loaned_publisher_1080p "rclcpp" "std_msgs")
target_link_libraries(loaned_publisher_1080p "${cpp_typesupport_target}" ${OpenCV_LIBS})
target_include_directories(loaned_publisher_1080p PUBLIC ${OpenCV_INCLUDE_DIRS})

# --- Loaned Subscriber (Zero-Copy) ---
add_executable(loaned_subscriber_1080p src/loaned_subscriber_1080p.cpp)
target_compile_features(loaned_subscriber_1080p PUBLIC cxx_std_17)
ament_target_dependencies(loaned_subscriber_1080p "rclcpp" "std_msgs")
target_link_libraries(loaned_subscriber_1080p "${cpp_typesupport_target}" ${OpenCV_LIBS})
target_include_directories(loaned_subscriber_1080p PUBLIC ${OpenCV_INCLUDE_DIRS})

# --- Standard Publisher (sensor_msgs/Image) ---
add_executable(standard_publisher_1080p src/standard_publisher_1080p.cpp)
target_compile_features(standard_publisher_1080p PUBLIC cxx_std_17)
ament_target_dependencies(standard_publisher_1080p "rclcpp" "sensor_msgs")
target_link_libraries(standard_publisher_1080p ${OpenCV_LIBS})
target_include_directories(standard_publisher_1080p PUBLIC ${OpenCV_INCLUDE_DIRS})

# --- Standard Subscriber (sensor_msgs/Image) ---
add_executable(standard_subscriber_1080p src/standard_subscriber_1080p.cpp)
target_compile_features(standard_subscriber_1080p PUBLIC cxx_std_17)
ament_target_dependencies(standard_subscriber_1080p "rclcpp" "sensor_msgs")
target_link_libraries(standard_subscriber_1080p ${OpenCV_LIBS})
target_include_directories(standard_subscriber_1080p PUBLIC ${OpenCV_INCLUDE_DIRS})

# ============================================================================
# INSTALLATION / INSTALLAZIONE
# ============================================================================

# Install SIM library
install(TARGETS sim_transport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install headers
install(DIRECTORY include/
  DESTINATION include)

install(TARGETS 
  sim_publisher_1080p
  sim_subscriber_1080p
  sim_publisher_nofb_1080p
  sim_subscriber_nofb_1080p
  loaned_publisher_1080p
  loaned_subscriber_1080p
  standard_publisher_1080p
  standard_subscriber_1080p
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install video folder (empty, user adds video)
install(DIRECTORY video/
  DESTINATION share/${PROJECT_NAME}/video
)

# Install docs
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

# Install assets (GIFs)
install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

# Install guides
install(DIRECTORY guides/
  DESTINATION share/${PROJECT_NAME}/guides
)

# Install SIM library (for reuse)
install(DIRECTORY sim_library/
  DESTINATION share/${PROJECT_NAME}/sim_library
)

# ============================================================================
# TESTING
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
