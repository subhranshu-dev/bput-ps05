from setuptools import setup

package_name = 'lidar_processor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BPUT PS05 Team',
    maintainer_email='subhransu25112005@gmail.com',
    description='LiDAR filtering/downsampling, health monitoring and test-harness fault injection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_processor = lidar_processor.node:main',
        ],
    },
)
