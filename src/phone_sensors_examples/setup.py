import os
from glob import glob
from setuptools import find_packages, setup

package_name = "phone_sensors_examples"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="victor",
    maintainer_email="victor.talpaert@gmail.com",
    description="Examples for phone_sensors",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
    },
)
