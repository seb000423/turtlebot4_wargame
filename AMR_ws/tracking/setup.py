from setuptools import find_packages, setup


package_name = "tracking"


setup(
    name=package_name,
    version="0.0.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="Seungeon Boo",
    maintainer_email="seb000423@gmail.com",

    description="Tracking mission package",

    license="Apache-2.0",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            "azimuth = tracking.azimuth_tracker:main",
            "target = tracking.target_nav:main",
            "follow = tracking.Tb4_follower:main",
            "mission_manager = tracking.mission_manager:main",
            "random_patrol = tracking.random_patrol:main",
        ],
    },
)
