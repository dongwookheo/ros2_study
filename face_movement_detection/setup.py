from setuptools import find_packages, setup

package_name = "face_movement_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mint",
    maintainer_email="hdwook3918@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "face_movement_detector = face_movement_detection.face_movement_detector:main",
            "detector_client = face_movement_detection.client:main",
        ],
    },
)
