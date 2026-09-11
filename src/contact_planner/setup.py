from setuptools import find_packages, setup

package_name = 'contact_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyeokey',
    maintainer_email='hyeokey@todo.todo',
    description='/scan_3D를 RANSAC으로 평면검출 후 접촉점/pre-contact 지점을 계산해 /piper/target_pose로 발행 (2026-09-07: plane_detector 병합)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'contact_planner_node = contact_planner.contact_planner_node:main',
        ],
    },
)
