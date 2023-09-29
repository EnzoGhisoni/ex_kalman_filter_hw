import os
from setuptools import setup
from glob import glob

package_name = 'ex_kalman_filter_hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enzo',
    maintainer_email='enzo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ex_kalman_node = ex_kalman_filter_hw.ex_kalman_filter:main',
            'republish_bag_node = ex_kalman_filter_hw.republish_bag:main',
        ],
    },
)
