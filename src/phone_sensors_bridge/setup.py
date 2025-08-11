import os
from glob import glob
from setuptools import find_packages, setup

package_name = "phone_sensors_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("lib", "templates"), glob("templates/*.html")),
        (os.path.join("lib", "static"), glob("templates/static/*.js") + glob("templates/static/*.css")),
    ],
    install_requires=["setuptools", "flask-socketio", "gevent-websocket", "numpy", "opencv-python"],
    zip_safe=True,
    maintainer="victor",
    maintainer_email="victor.talpaert@gmail.com",
    description="Serve a webpage to open from a mobile device. The phone's browser will send video, imu and gps data back to the node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["server = phone_sensors_bridge.server:main"],
    },
    scripts=["scripts/generate_dev_certificates.sh"],
)
