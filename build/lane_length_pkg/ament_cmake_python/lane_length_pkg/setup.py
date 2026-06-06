from setuptools import find_packages
from setuptools import setup

setup(
    name='lane_length_pkg',
    version='0.0.0',
    packages=find_packages(
        include=('lane_length_pkg', 'lane_length_pkg.*')),
)
