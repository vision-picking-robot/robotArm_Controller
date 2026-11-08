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
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML>=6'],
    zip_safe=True,
    maintainer='naeun',
    maintainer_email='skdms8827@s3i.co.kr',
    description='Doosan E0509 pick and place motion with floor safety limits',
    license='Apache-2.0',
    extras_require={'test': ['pytest>=7']},
    entry_points={
        'console_scripts': [
            'motion_server = warehouse_motion.motion_server:main',
            'picking_coordinator = warehouse_motion.picking_coordinator:main',
            'rfid_db_node = warehouse_motion.db.rfid_db_node:main',
            'warehouse_mock = warehouse_motion.mock_demo:main',
            'warehouse_scenario = warehouse_motion.mock_scenario:main',
        ],
    },
)
