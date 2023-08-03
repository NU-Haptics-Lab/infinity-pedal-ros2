cmake_minimum_required(VERSION 3.8)
project(infinity_pedal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(/src)

FILE(GLOB src_files 
      ./src/*.cpp
      ./src/*.hpp
      ./src/*.h
    )


set(NAME infinity_pedal)
add_executable(${NAME} ${src_files})
ament_target_dependencies(${NAME} rclcpp std_msgs)
target_link_libraries(${NAME} -ludev)
install(TARGETS
  ${NAME}
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()