from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'streamlit_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='loki',
    maintainer_email='bigrobinson@gmail.com',
    description='A streamlit UI with socket interfaces to ROS2 publisher/subscriber',
    license='Unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'streamlit_publisher = streamlit_ui.streamlit_publisher_node:main',
            'streamlit_subscriber = streamlit_ui.streamlit_subscriber_node:main',
        ],
    },
)
