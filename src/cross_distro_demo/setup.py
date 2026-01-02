from setuptools import setup

package_name = 'cross_distro_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pc_launch.py', 'launch/pi_launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A cross-distro demo for communication between PC and Raspberry Pi using ROS2.',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pc_publisher = cross_distro_demo.pc_publisher:main',
            'pi_subscriber = cross_distro_demo.pi_subscriber:main',
        ],
    },
)