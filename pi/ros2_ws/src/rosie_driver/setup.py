from setuptools import find_packages, setup

package_name = 'rosie_driver'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosie',
    maintainer_email='rosie@local',
    description='ROSie Neato D6 Pi-side ROS 2 driver node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_node = rosie_driver.driver_node:main',
        ],
    },
)
