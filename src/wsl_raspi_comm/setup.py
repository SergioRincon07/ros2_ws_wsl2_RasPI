from setuptools import find_packages, setup

package_name = 'wsl_raspi_comm'

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
    maintainer='sergio',
    maintainer_email='sergiorincon50@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = wsl_raspi_comm.talker:main',
            'listener = wsl_raspi_comm.listener:main',
            'ping_node = wsl_raspi_comm.ping_node:main',
            'pong_node = wsl_raspi_comm.pong_node:main',
            'service_server = wsl_raspi_comm.service_server:main',
            'service_client = wsl_raspi_comm.service_client:main',
        ],
    },
)
