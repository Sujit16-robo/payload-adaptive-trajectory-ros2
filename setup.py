from setuptools import setup
import os
from glob import glob

package_name = 'payload_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sp16',
    maintainer_email='pattanaiks627@gmail.com',
    description='Inertia-adaptive trajectory scaling demo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'traj_publisher = payload_arm.traj_publisher:main',
            'interceptor_node = payload_arm.interceptor_node:main',
            'data_logger = payload_arm.data_logger:main',
            'trajectory_executor = payload_arm.trajectory_executor:main',
        ],
    },
)