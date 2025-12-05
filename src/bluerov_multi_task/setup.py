from setuptools import setup

package_name = 'bluerov_multi_task'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Required by ament so the package is discoverable
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Install package.xml
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        ('share/' + package_name + '/launch', [
            'launch/two_bluerovs.launch.py',
            'launch/single_bluerov_control.launch.py',
        ]),

        # Install scenarios
        ('share/' + package_name + '/scenarios', [
            'scenarios/bluerov2_heavy_multi.scn',
            'scenarios/two_bluerovs.scn',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elex',            # change if you want
    maintainer_email='you@example.com',  # change to your email or leave placeholder
    description='Multi-BlueROV Stonefish tasks and scenarios',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Your multi-robot follower node
            'bluerov_multi_follower = bluerov_multi_task.bluerov_multi_follower:main',
            'cmd_vel_to_pwm = bluerov_multi_task.cmd_vel_to_pwm:main',
            'joystick = bluerov_multi_task.joystick:main',
        ],
    },
)

