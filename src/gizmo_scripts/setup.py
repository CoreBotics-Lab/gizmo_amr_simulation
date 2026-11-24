from setuptools import find_packages, setup #type: ignore

package_name = 'gizmo_scripts'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CoreBotics-Lab',
    maintainer_email='hayisyed@gmail.com',
    description='Python vision processing, AprilTag waypoint management, and Nav2 mission scripts for Gizmo AMR',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'ros_cv = gizmo_scripts.00_ros_with_computer_vision:main',
            'color_tracker = gizmo_scripts.01_color_contour_tracker:main',
            'line_follower = gizmo_scripts.02_line_follower:main',
            'apriltag_tracker = gizmo_scripts.03_apriltag_tracker:main',
        ],
    },
)
