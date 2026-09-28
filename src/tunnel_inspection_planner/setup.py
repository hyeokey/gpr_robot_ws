from glob import glob

from setuptools import find_packages, setup

package_name = "tunnel_inspection_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hyeokey",
    maintainer_email="msol62@krri.re.kr",
    description=(
        "Automatic multi-segment tunnel arch inspection coverage planner "
        "(read-only dry-run reachability + live sequencing structure)."
    ),
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "coverage_planner_node = tunnel_inspection_planner.coverage_planner_node:main",
            "sim_tf_crosscheck = tunnel_inspection_planner.sim_tf_crosscheck:main",
            "segment_sequencer_node = tunnel_inspection_planner.segment_sequencer_node:main",
        ],
    },
)
