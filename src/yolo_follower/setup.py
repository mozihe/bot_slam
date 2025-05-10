from setuptools import find_packages, setup

package_name = 'yolo_follower'

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
    maintainer='mozihe',
    maintainer_email='mozihe@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_follower_node = yolo_follower.yolo_follower_node:main',
            'image_recorder_node = yolo_follower.image_recorder_node:main',
            'yolo_tracker_node = yolo_follower.yolo_tracker_node:main'
        ],
    },
)
