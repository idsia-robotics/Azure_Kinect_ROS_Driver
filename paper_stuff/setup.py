from setuptools import setup
import glob

package_name = 'paper_stuff'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + "/launch", glob.glob("launch/*")),
        ('share/' + package_name + "/models", glob.glob("models/*")),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sermas',
    maintainer_email='sermas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lstm_node = paper_stuff.lstm_node:main',
            'rf_node = paper_stuff.rf_node:main',
            'features_debug_node = paper_stuff.features_debug_node:main',
            'skeleton_to_rgb = paper_stuff.skeleton_to_rgb:main',
            'data_collector_node = paper_stuff.data_collector_node:main',
            'rm_demo = paper_stuff.rm_demo:main'
        ],
    },
)
