from setuptools import find_packages, setup

package_name = 'piper_controller'

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
    description='Piper 로봇팔 자동 모드 컨트롤러: /piper/target_pose 구독, PyBullet IK + MIT 모드로 실제 팔 구동',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'piper_controller_node = piper_controller.piper_controller_node:main',
            'push_forward_node = piper_controller.push_forward_node:main',
        ],
    },
)
