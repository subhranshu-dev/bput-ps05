from setuptools import setup

package_name = 'aeronav_simulation'

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
    description='Gazebo world tooling: ground tracking station sensor model, dynamic obstacle and scenario events',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ground_tracker = aeronav_simulation.ground_tracker:main',
            'scenario_events = aeronav_simulation.scenario_events:main',
        ],
    },
)
