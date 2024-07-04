from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'akrobat'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'launch', 'include'), glob('launch/include/*.launch.*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'stl', 'visual'), glob('stl/visual/*.stl')),
        (os.path.join('share', package_name, 'stl', 'collision'), glob('stl/collision/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benjamin',
    maintainer_email='benjamin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'akrobat = {package_name}.akrobat:main'
        ],
    },
)
