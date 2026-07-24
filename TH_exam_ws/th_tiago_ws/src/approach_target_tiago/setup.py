from setuptools import find_packages, setup

package_name = 'approach_target_tiago'

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
            'approach_target_node = approach_target_tiago.approach_target_node:main',
            'distance_detection_node = approach_target_tiago.distance_detection_node:main',
        ],
    },
)
