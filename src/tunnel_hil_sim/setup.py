from glob import glob
from setuptools import find_packages, setup


package_name = "tunnel_hil_sim"


setup(
    name=package_name,
    version="0.3.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.rviz")),
        (f"share/{package_name}/worlds", glob("worlds/*.sdf")),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gpr_robot_ws maintainer",
    maintainer_email="user@example.com",
    description="Gazebo tunnel and dual-LiDAR environment for Piper HIL development.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sim_pointcloud_adapter = tunnel_hil_sim.sim_pointcloud_adapter:main",
            "joint_mirror_node = tunnel_hil_sim.joint_mirror_node:main",
            "platform_control_node = tunnel_hil_sim.platform_control_node:main",
        ],
    },
)
