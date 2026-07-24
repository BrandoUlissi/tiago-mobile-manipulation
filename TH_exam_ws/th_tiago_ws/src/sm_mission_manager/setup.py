from setuptools import find_packages, setup

package_name = 'sm_mission_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luca',
    maintainer_email='luca.bachetti.spurio@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sm_mission_manager=sm_mission_manager.sm_mission_manager:main",
            "odom_check_node=sm_mission_manager.odom_check_node:main",

        ],
    },
)
