"""
Install the adaptive pure pursuit library
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["adaptive_pure_pursuit"], package_dir={"": "src"})

setup(**d)
