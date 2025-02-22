import os
from glob import glob

from setuptools import setup

package_name = "ros2_dream_base_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        # Install launch files
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include launch, meshes, urdf, worlds, config, etc.
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        # Include any additional directories as needed
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rico Jia",
    maintainer_email="ruotongjia2020@u.northwestern.edu",
    description="A differential drive model",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Define your Python nodes here
            # Example:
            # 'my_python_node = ros2_dream_base_description.my_python_node:main',
        ],
    },
)
