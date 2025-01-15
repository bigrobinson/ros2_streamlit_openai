from setuptools import find_packages, setup

package_name = 'openai_ros2'

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
    maintainer='bigrobinson',
    maintainer_email='bigrobinson@gmail.com',
    description='simple ChatGPT/ROS2 integration',
    license='The Unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent = openai_ros2.agent_node:main',
        ],
    },
)
