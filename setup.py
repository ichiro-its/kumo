from setuptools import setup

package_name = 'kumo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'websockets'
    ],
    zip_safe=True,
    maintainer='Alfi Maulana',
    maintainer_email='alfi.maulana.f@gmail.com',
    description='WebSocket bridge for ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = kumo.bridge:main'
        ],
    },
)
