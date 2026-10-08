from setuptools import setup

package_name = 'px4_interface'

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
    description='PX4 offboard interface over uXRCE-DDS (arm, mode, setpoints, land)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'px4_interface = px4_interface.node:main',
            'smoke_flight = px4_interface.smoke_flight:main',
        ],
    },
)
