from glob import glob
from setuptools import find_packages, setup


package_name = "rokae_motion"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/config",
            glob("config/*.json"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="niic",
    maintainer_email="niic@localhost",
    description=(
        "Python MoveAbsJ action clients and dual-arm motion programs."
    ),
    license="Proprietary",
    entry_points={
        "console_scripts": [
            (
                "moveabsj_client = "
                "rokae_motion.ros_dual_moveabsj_client:main"
            ),
            (
                "dual_arm_program = "
                "rokae_motion.run_moveabsj_program:main"
            ),
        ],
    },
)
