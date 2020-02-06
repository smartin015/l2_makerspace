from setuptools import setup
from os import environ

package_name = environ['L2NAME']
node_files = "$NODES"
nodes = [node.split('.')[0] for node in node_files.split(' ')]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FabricateIO',
    maintainer_email='contact@fabricate.io',
    description='See package.xml',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '%s = %s.%s:main' % (node, package_name, node)
            for node in nodes
        ],
    },
)
