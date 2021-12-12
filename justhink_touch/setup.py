from setuptools import setup

package_name = 'justhink_touch'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='utku',
    maintainer_email='utku.norman@epfl.ch',
    description='TODO: Package description',
    license='MIT',
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'convert_touch = justhink_touch.touch_converter_node:main',
        ],
    },

)
