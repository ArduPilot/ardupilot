import os
from glob import glob
from setuptools import setup

package_name = 'ardupilot_dds_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.parm")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@ardupilot.org',
    description='Tests for the ArduPilot AP_DDS library',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "time_listener = ardupilot_dds_tests.time_listener:main",
            "plane_waypoint_follower = ardupilot_dds_tests.plane_waypoint_follower:main",
        ],
    },
)
