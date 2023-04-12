from setuptools import setup
import os
from os.path import join
from glob import glob

package_name = 'krytn_cafe'
package_share = join('share', package_name)
data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),

    ('share/' + package_name, ['package.xml']),

    (join('share', package_name, 'launch'), glob('launch/*launch.py')),
    (join(package_share, 'config'), glob('config/*.yaml')),

    (join(package_share, 'models/krytn/'),
     glob('models/krytn/*.urdf') + glob('models/krytn/*.xacro')
     + glob('models/krytn/model.config')),

    (join(package_share, 'models/krytn/meshes/'),
     glob('models/krytn/meshes/*.dae'))
]
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='badwolf.johnnyv@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = krytn_cafe.my_node:main'
        ],
    },
)
