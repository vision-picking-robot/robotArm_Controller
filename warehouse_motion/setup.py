from glob import glob

from setuptools import find_packages, setup

package_name = 'warehouse_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naeun',
    maintainer_email='skdms8827@s3i.co.kr',
    description='Doosan E0509 pick and place motion with floor safety limits',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_server = warehouse_motion.motion_server:main',
            'rfid_db_node = warehouse_motion.db.rfid_db_node:main',
        ],
    },
)
