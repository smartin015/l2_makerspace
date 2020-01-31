from setuptools import setup

package_name = 'l2_depth_imagery'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FabricateIO',
    maintainer_email='contact@fabricate.io',
    description='depth camera image node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = node:main',
        ],
    },
)
