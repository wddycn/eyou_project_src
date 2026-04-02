from setuptools import find_packages, setup
import os  # 新加
from glob import glob  # 新加

package_name = 'eyou_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # 新加
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),    # 新加
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')), # 新加

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ycn',
    maintainer_email='2422581995@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)