install(
  FILES msg.h.template srv.h.template
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS gen_cpp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
