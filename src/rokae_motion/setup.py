from glob import glob
from pathlib import Path
from setuptools import find_packages, setup


package_name = "rokae_motion"


def behavior_tree_data_files():
    """Install YAML programs while preserving their subdirectories."""
    source_root = Path("behavior_trees")
    return [
        (
            str(Path("share") / package_name / path.parent),
            [str(path)],
        )
        for path in source_root.rglob("*.yaml")
    ]


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
    ] + behavior_tree_data_files(),
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="niic",
    maintainer_email="tolbayoma6@gmail.com",
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
            (
                "bt_runner = "
                "rokae_motion.run_bt:main"
            ),
            (
                "bt_control = "
                "rokae_motion.control:main"
            ),
        ],
    },
)
