cmake_minimum_required(VERSION 3.8)
project(dual_sharpa_wave)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RecoverDefault.action"
  "srv/MitGains.srv"
)

# rosidl already registers and installs the generated Python package. Install
# the driver modules beside it without registering the same package twice.
# The source __init__.py only contains a docstring; retain rosidl's initializer.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY dual_sharpa_wave/
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  PATTERN "__init__.py" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  script/hand_node
  script/tactile_viewer.py
  script/gui_control.py
  script/adjust_mit_gains_gui.py
  script/sine_control.py
  script/step_control.py
  script/move_to_default_pose.py
  script/read_mit_settings.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch urdf rviz third_party docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sharpa test
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    TIMEOUT 180
    ENV
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_DOMAIN_ID=173
      "ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/ros_home"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      OPENBLAS_NUM_THREADS=1
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
