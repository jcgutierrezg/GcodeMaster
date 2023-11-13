from setuptools import setup

package_name = 'gcode_executor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Camilo Gutierrez',
    maintainer_email='jc.gutierrezg@uniandes.edu.co',
    description='This package contains a custom made gcode file interpreter to send movement instructions to ROS2 in order to control Robocols arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gcode_reader = gcode_executor.gcode_reader:main',
            'gui = gcode_executor.gui:main'
        ],
    },
)
