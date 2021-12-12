from setuptools import setup

package_name = 'justhink_situation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Utku Norman',
    maintainer_email='utku.norman@epfl.ch',
    description='ROS wrapper for JUSThink Human-Robot Pedagogical Activity',
    license='MIT',
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    entry_points={
        'console_scripts': [
            'show_situation = justhink_situation.justhink_situation_node:main',
        ],
    },
)
