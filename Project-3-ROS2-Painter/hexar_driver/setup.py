from setuptools import setup

package_name = 'hexar_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Will Ward',
    maintainer_email='willward1912@gmail.com',
    description='Controlling a hexar robot with velocity commands published over turtlesim velocity topic',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bringup_hexar = hexar_driver.bringup_hexar:main'
        ],
    },
)
