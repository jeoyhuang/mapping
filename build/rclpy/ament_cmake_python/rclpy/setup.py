import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='rclpy',
    version='5.0.0',
    packages=find_packages(
        include=('rclpy', 'rclpy.*')),
)
