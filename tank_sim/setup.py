from setuptools import setup

package_name = 'tank_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maddie Edmondson',
    maintainer_email='maddieedmondson19@gmail.com',
    description='Tank simulation package with Gazebo and ROS 2 Jazzy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_to_gazebo = tank_sim.tf_to_gazebo:main'
        ],
    },
)


