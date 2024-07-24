from setuptools import find_packages, setup

package_name = 'build_semantic_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/srv', ['srv/SetSlam.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sg',
    maintainer_email='shugaoliu@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'build_semantic_map = build_semantic_map.build_semantic_map_service:main',
            'build_semantic_map_client = build_semantic_map.build_semantic_map_client:main',
        ],
    },
)

