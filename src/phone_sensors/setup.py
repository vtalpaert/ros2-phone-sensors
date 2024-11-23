import os
from glob import glob
from setuptools import find_packages, setup

package_name = "phone_sensors"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("lib", "templates"), glob("templates/*.html")),
        (os.path.join("lib", "static"), glob("templates/static/*.js")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="victor",
    maintainer_email="victor.talpaert@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["server = phone_sensors.server:main"],
    },
)
