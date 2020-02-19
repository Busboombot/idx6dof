#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

from setuptools import find_packages
import uuid
import imp


try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup


if sys.argv[-1] == 'publish':
    os.system('python setup.py sdist upload')
    sys.exit()

with open(os.path.join(os.path.dirname(__file__), 'README.rst')) as f:
    readme = f.read()

packages = find_packages()

classifiers = [
    'Development Status :: 4 - Beta',
    'Intended Audience :: Developers',
    'License :: OSI Approved :: MIT License',
    'Operating System :: OS Independent',
    'Programming Language :: Python',
    'Topic :: Software Development :: Libraries :: Python Modules',
]

setup(
    name='trajectory',
    version=0.01,
    description='Motion control trajectory planning and comminication with an embedded controller',
    long_description=readme,
    packages=packages,
    install_requires=[
        
    ],
    author="Eric Busboom",
    author_email='eric@civicknowledge.com',
    url='https://github.com/CivicKnowledge/trajectory.git',
    license='MIT',
    classifiers=classifiers,
    entry_points={
        'console_scripts': [
            'robot_joystick=trajectory.cli:run_joystick',
            'joy_planner=trajectory.cli:run_joy_planner'
        ],
    },
)
