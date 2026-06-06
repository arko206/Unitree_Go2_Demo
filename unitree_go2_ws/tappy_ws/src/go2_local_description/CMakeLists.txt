cmake_minimum_required(VERSION 3.10)
project(go2_local_description)
find_package(ament_cmake REQUIRED)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf meshes dae launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
