#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dcsl_swarm_director'],
    package_dir={'': 'src'}
)

setup(**d)
