cmake_minimum_required(VERSION 3.0.2)
project(antropomorphic_project)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()  

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/generate_matrixes.py
  src/fk_antropomorphic_arm.py
  src/ik_antropomorphic_arm.py
  src/rviz_marker.py
  src/move_joints.py
  src/ellipsoidal_motion.py
  src/antropomorphic_end_effector_mover.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
