from setuptools import setup

package_name = 'turtlebro_actions'
packages = ['turtlebro_actions']

setup(
    name=package_name,
    version='0.1.0',
    packages=packages,
    package_dir={'turtlebro_actions': 'turtlebro_actions'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Action-серверы, клиенты и вспомогательные утилиты TurtleBro для ROS 2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_server = turtlebro_actions.move_server:main',
            'move_client = turtlebro_actions.move_client:main',
            'rotate_server = turtlebro_actions.rotate_server:main',
            'rotate_client = turtlebro_actions.rotate_client:main',
            'photo_service = turtlebro_actions.photo_service:main',
            'radio_command = turtlebro_actions.radio:main',
        ],
    },
)
