from setuptools import find_packages, setup

package_name = 'hmi_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eddyswens',
    maintainer_email='pyhalof.egor@yandex.ru',
    description='Package to control robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hmi_control_node = hmi_control.hmi_control_node:main'
        ],
    },
)
