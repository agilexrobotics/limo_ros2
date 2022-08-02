from glob import glob
from setuptools import setup,find_packages
import os
package_name = 'voice_control'

plugins = glob("Plugins/**/**.**")
plugin_path = [(os.path.join("share",package_name,os.path.dirname(c)), [c]) for c in plugins]
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={'Laplace':["PluginSystem/*","PluginSystem/common/*"]},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        *plugin_path
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='originsun@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "voice_control = voice_control.main:main"
        ],
    },
)
