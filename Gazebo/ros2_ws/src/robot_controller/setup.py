from setuptools import setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'robot_controller.robot_controller',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TwojeImie',
    maintainer_email='twojemail@example.com',
    description='Pakiet do sterowania robotem w ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robot_controller = robot_controller.robot_controller:main',
        ],
    },
)

