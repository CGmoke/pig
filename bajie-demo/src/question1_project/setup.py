from glob import glob

from setuptools import setup

package_name = 'bajie_tasks'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moke',
    maintainer_email='moke@localhost',
    description='八界杯初赛题目一：关节分步驱动',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'q1_drive = bajie_tasks.q1_drive:main',
        ],
    },
)
