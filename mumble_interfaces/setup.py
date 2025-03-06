from setuptools import setup, find_packages

package_name = "mumble_interfaces"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rico",
    maintainer_email="ruotongjia2020@u.northwestern.edu",
    description="Mumble Interfaces Package for ROS2",
    license="BSD-2",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
