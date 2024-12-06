from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'for_open_scout'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f"share/{package_name}/launch", glob(os.path.join('launch', '*launch.[pxy][yml]*'))),
        (f"share/{package_name}/models", glob(os.path.join('models', '*.*'))),
        (f"share/{package_name}/rviz", glob(os.path.join('rviz', '*.[rviz]*'))),
        (f"share/{package_name}/urdf", glob(os.path.join('urdf', '*.*'))),
        (f"share/{package_name}/worlds", glob(os.path.join('worlds', '*.[world]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='contact@gauthampdas.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
