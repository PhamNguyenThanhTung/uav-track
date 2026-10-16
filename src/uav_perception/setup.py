from setuptools import find_packages, setup

package_name = 'uav_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/botsort.yaml', 'config/fastdds_images.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tungt',
    maintainer_email='tungt@example.com',
    description='Real-time YOLO26n and BoT-SORT perception package for UAV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = uav_perception.yolo_detector:main',
        ],
    },
)
