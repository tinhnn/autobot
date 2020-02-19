import glob
import subprocess
from setuptools import setup, find_packages, Extension


def build_libs():
    subprocess.call(['cmake', '.'])
    subprocess.call(['make'])
    

build_libs()


setup(
    name='autobot',
    version='0.1.0',
    description='An open-source robot based on raspberry pi',
    packages=find_packages(),
    install_requires=[
        'pigpio',
    ],
)
