#!/usr/bin/env python3
from setuptools import setup, find_packages

setup(
    name='winch_control',
    version='0.0.0',
    # find all packages under src/
    packages=find_packages('src'),
    package_dir={'': 'src'},
    install_requires=[
        'dynamixel_sdk',
    ],
)
