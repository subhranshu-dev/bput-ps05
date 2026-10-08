from setuptools import setup

package_name = 'aeronav_description'

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
    description='Static TF2 sensor-mount frames for the PS05 X500 vehicle',
    license='MIT',
    entry_points={
        'console_scripts': [
            'static_frames = aeronav_description.static_frames:main',
        ],
    },
)
