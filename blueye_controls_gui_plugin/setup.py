from setuptools import setup
import os
from glob import glob

package_name = 'blueye_controls_gui_plugin'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.ui'))
    ],
    install_requires=['setuptools', 'rqt_gui', 'rqt_gui_py'], # NEW
    zip_safe=True,
    maintainer='nadir',
    maintainer_email='nadir.kapetanovic@fer.hr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
