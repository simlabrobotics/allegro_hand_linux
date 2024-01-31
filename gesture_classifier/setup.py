from setuptools import find_packages, setup

package_name = 'gesture_classifier'
submodules = "gesture_classifier/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naderahmed',
    maintainer_email='ndr.ahmed1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_classifier = gesture_classifier.gesture_classifier:main',
        ],
    },
)
