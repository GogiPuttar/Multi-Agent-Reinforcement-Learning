from setuptools import find_packages, setup

package_name = 'marl'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/train.launch.py']),
    ],
    install_requires=['setuptools',
                      'numpy',
                      'tensorflow',
                      'rclpy'],
    zip_safe=True,
    maintainer='adityanair',
    maintainer_email='aditya.nair0123@gmail.com',
    description='ROS2 package for Multi-Agent Reinforcement Learning with Turtlebots',
    license='APLv2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_service = marl_turtlebots.train_service:main',
        ],
    },
)
