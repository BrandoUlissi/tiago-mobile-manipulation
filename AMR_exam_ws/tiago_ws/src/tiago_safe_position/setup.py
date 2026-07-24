from setuptools import find_packages, setup
package_name = 'tiago_safe_position'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/safe_position_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brando',
    maintainer_email='tuo@email.it',
    description='Pacchetto per mettere Tiago in posizione di sicurezza',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tiago_arm_position = tiago_safe_position.tiago_arm_position:main',
            'tiago_head_position = tiago_safe_position.tiago_head_position:main',
        ],
    },
)

