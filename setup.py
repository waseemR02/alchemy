from setuptools import setup

package_name = 'alchemy'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='waseem.riaz.9999@gmail.com',
    description='ROS2 package for publishing all related info from bio to separate topics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'alchemy_node = alchemy.alchemy_node:main',
            'pseudo_node = alchemy.pseudo_node:main'
        ],
    },
)
