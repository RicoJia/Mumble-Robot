from setuptools import find_packages, setup

package_name = 'mumble_physical_runtime'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricojia',
    maintainer_email='ricoruotongjia@gmail.com',
    description='Small Package For Mumble Robot Physical I/O',
    license='BSD-2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_interface = mumble_physical_runtime.serial_interface:main',
            'test_service_caller = mumble_physical_runtime.test_service_caller:main',
        ],
    },
)
