from setuptools import setup  # , find_packages

package_name = 'justhink_situation'
# submodules = "justhink_situation/justhink_world"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # , submodules],
    # packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='utku',
    maintainer_email='utku.norman@epfl.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'show_situation = justhink_situation.justhink_situation_node:main',
        ],
    },
)
