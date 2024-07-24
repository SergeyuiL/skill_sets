from setuptools import find_packages, setup

package_name = 'py_robot_skills'

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
    maintainer='drl',
    maintainer_email='drl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # "skill_simple_server = py_robot_skills.skill_simple_server:main",
            # "skill_simple_client = py_robot_skills.skill_simple_client:main",
            # "skill_action_server = py_robot_skills.skill_action_server:main",
            # "skill_action_client = py_robot_skills.skill_action_client:main",
            "skill_server = py_robot_skills.skill_server:main"
        ],
    },
)
