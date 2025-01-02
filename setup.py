from setuptools import find_packages, setup

package_name = 'data_visualisation_pkg'

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
    maintainer='polyblank',
    maintainer_email='achermann@campus.tu-berlin.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'plant_data_visualizer = data_visualisation_pkg.plant_data_visualizer:main',
        ],
    },
    package_dir={'': 'src'},
    package_data={'': ['../data_collection_pkg']},
)
