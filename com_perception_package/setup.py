from setuptools import find_packages, setup

package_name = 'com_perception_package'

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
    maintainer='rui',
    maintainer_email='rui@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # send_scan.py must define a main() that starts your SendScanNode
            'send_scan_node = com_perception_package.send_scan:main',
            # recv_results.py must define a main() that starts your RecvResultsNode
            'recv_results_node = com_perception_package.recv_results:main',
            # perception_node.py must define a main() that starts your PerceptionNode
            'perception_node = com_perception_package.perception:main',
        ],
    },
)
