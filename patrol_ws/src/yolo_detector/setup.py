from setuptools import find_packages, setup

package_name = 'yolo_detector'

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
    maintainer='Seungeon Boo',
    maintainer_email='seb000423@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_detector.yolo_detector:main',
            'yolo_seg_detector = yolo_detector.yolo_seg_detector:main',
            'yolo2 = yolo_detector.yolo_seg2_detector:main',
            'bbox = yolo_detector.bbox:main',
            'bbox_origin = yolo_detector.bbox_origin:main',
            'bbox3 = yolo_detector.bbox3:main',
            'bbox4 = yolo_detector.bbox4:main',
            'first = yolo_detector.first_bbox:main',
            'phone_cam_detector = yolo_detector.phone_cam_detector:main',
        ],
    },
)
