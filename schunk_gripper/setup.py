from setuptools import find_packages, setup

package_name = 'schunk_gripper'

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
    maintainer='iris-hyundai',
    maintainer_email='jannet5234@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "alice_gripper = schunk_gripper.alice_gripper:main",
            "bob_gripper= schunk_gripper.bob_gripper:main"
        ],
    },
)
