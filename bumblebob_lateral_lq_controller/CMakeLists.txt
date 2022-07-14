cmake_minimum_required(VERSION 2.8.3)
project(bumblebob_lateral_lq_controller)

find_package(catkin REQUIRED COMPONENTS
        rospy
        dynamic_reconfigure
    )



#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  config/Dynamic.cfg
)

catkin_package()

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest(tests/lateral_lqr_controller_node.test)
# endif()

# catkin_python_setup()
# catkin_install_python(PROGRAMS src/bumblebob_lateral_lq_controller/lateral_lqr_controller_node.py
#                       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})