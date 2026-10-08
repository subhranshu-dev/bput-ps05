from setuptools import setup

package_name = 'landing_validator'

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
    description='LiDAR landing-surface and clearance validation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'validator_node = landing_validator.node:main',
        ],
    },
)
