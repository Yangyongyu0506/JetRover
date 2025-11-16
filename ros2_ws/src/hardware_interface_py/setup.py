from setuptools import find_packages, setup
from glob import glob

package_name = 'hardware_interface_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*')),
        (f'share/{package_name}/config', glob('config/*')),
        (f'share/{package_name}/data', glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='yang-yy24@mails.tsinghua.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ESP_agent = hardware_interface_py.nodes.ESP_agent:main'
        ],
    },
)
