^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gencpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2014-05-07)
------------------
* add architecture_independent flag in package.xml (`#19 <https://github.com/ros/gencpp/issues/19>`_)

0.5.1 (2014-02-24)
------------------
* use catkin_install_python() to install Python scripts (`#18 <https://github.com/ros/gencpp/issues/18>`_)
* add 'u' suffix to unsigned enum values to avoid compiler warning (`#16 <https://github.com/ros/gencpp/issues/16>`_)

0.5.0 (2014-01-29)
------------------
* remove __connection_header from message template (`#3 <https://github.com/ros/gencpp/issues/3>`_)

0.4.16 (2014-01-27)
-------------------
* fix warning about empty message definition (`ros/ros_comm#344 <https://github.com/ros/ros_comm/issues/344>`_)

0.4.15 (2014-01-07)
-------------------
* python 3 compatibility
* fix generated code of message definition with windows line endings (`#6 <https://github.com/ros/gencpp/issues/6>`_)

0.4.14 (2013-08-21)
-------------------
* make gencpp relocatable (`ros/catkin#490 <https://github.com/ros/catkin/issues/490>`_)

0.4.13 (2013-06-18)
-------------------
* update message targets to depend on template
* update msg template to generate empty functions without warnings about unused variables (`#4 <https://github.com/ros/gencpp/issues/4>`_)

0.4.12 (2013-03-08)
-------------------
* fix handling spaces in folder names (`ros/catkin#375 <https://github.com/ros/catkin/issues/375>`_)

0.4.11 (2012-12-21)
-------------------
* first public release for Groovy
