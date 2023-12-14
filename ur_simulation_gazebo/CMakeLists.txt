cmake_minimum_required(VERSION 3.5)
project(ur_simulation_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(description test/test_description.py)
endif()

ament_package()
