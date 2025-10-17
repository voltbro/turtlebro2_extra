from setuptools import setup

package_name = 'turtlebro_actions'
python_package = 'turtlebro_actions_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[python_package],
    package_dir={python_package: 'turtlebro_actions_nodes'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Turtlebro action servers, clients, and helpers for ROS 2 Jazzy',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_server = turtlebro_actions_nodes.move_server:main',
            'move_client = turtlebro_actions_nodes.move_client:main',
            'rotate_server = turtlebro_actions_nodes.rotate_server:main',
            'rotate_client = turtlebro_actions_nodes.rotate_client:main',
            'photo_service = turtlebro_actions_nodes.photo_service:main',
            'radio_command = turtlebro_actions_nodes.radio:main',
        ],
    },
)
