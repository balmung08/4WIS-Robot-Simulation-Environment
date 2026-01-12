cmake_minimum_required(VERSION 3.0.2)
project(BARN_Env)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/map_info.py
    scripts/map_tf.py
    scripts/map_transfer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

