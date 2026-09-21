from glob import glob
from setuptools import find_packages, setup

package_name = "tunnel_wall_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hyeokey",
    maintainer_email="msol62@krri.re.kr",
    description=(
        "/lidar_1/scan_3D wall plane detection (RANSAC+SVD) and target-pose "
        "computation, RViz-marker-only, no real-arm control topics."
    ),
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "tunnel_wall_detector_node = tunnel_wall_detector.tunnel_wall_detector_node:main",
            "piper_target_relay_node = tunnel_wall_detector.piper_target_relay_node:main",
        ],
    },
)
