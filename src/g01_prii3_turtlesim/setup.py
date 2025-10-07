from setuptools import setup

package_name = 'g01_prii3_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sprint1_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='domenico',
    maintainer_email='tu_email@ejemplo.com',
    description='Nodo que dibuja el número de grupo con turtlesim',
    license='TODO',
    entry_points={
        'console_scripts': [
            'sprint1 = g01_prii3_turtlesim.sprint1:main',
        ],
    },
)

