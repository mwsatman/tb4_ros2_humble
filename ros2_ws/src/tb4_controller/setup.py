from setuptools import find_packages, setup

package_name = 'tb4_controller'

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
    maintainer='mwatma',
    maintainer_email='8626150+mwsatman@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_controller = tb4_controller.test_controller:main',
            'multirobot_test_controller = tb4_controller.multirobot_test_controller:main'
        ],
    },
)
