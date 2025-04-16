from setuptools import setup
from glob import glob


# def map_recursive_files(dest, directory):
#     return os.path.join(dest, directory), [y for x in os.walk(directory) for y in glob(os.path.join(x[0], '*'))]


package_name = 'turn_node'

setup(
    name=package_name,
    version='1.0.0',
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch*')),
        ('share/' + package_name + '/src', glob('src/*.py')),
        ('lib/' + package_name, glob('src/*.py'))
    ],
    description='Node for driving inside the road',
    author='Ivan Voevodskiy',
    author_email='ivanvoevodskij@gmail.com',
    license='GPLv3',
    entry_points={
        'console_scripts': [
            # TODO: Write an console scripts for this node
            'turn_node = turn_node.turn_node:main',
        ],
    },
)