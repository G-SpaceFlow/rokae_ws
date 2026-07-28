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


def config_data_files():
    """Install stable calibration and runtime configuration."""
    return [
        (
            str(Path("share") / package_name / "config"),
            [str(path)],
        )
        for path in Path("config").glob("*.yaml")
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
    ] + behavior_tree_data_files() + config_data_files(),
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="niic",
    maintainer_email="tolbayoma6@gmail.com",
    description=(
        "YAML behavior-tree orchestration for Rokae arm and hand actions."
    ),
    license="Proprietary",
    entry_points={
        "console_scripts": [
            (
                "bt_runner = "
                "rokae_motion.run_bt:main"
            ),
            (
                "bt_control = "
                "rokae_motion.control:main"
            ),
            (
                "vision_target_server = "
                "rokae_motion.vision_target_server:main"
            ),
            (
                "chassis_navigation = "
                "rokae_motion.chassis_navigation:main"
            ),
        ],
    },
)
