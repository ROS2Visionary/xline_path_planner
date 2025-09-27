from setuptools import find_packages
from setuptools import setup

setup(
    name='xline_path_planner',
    version='0.0.0',
    packages=find_packages(
        include=('xline_path_planner', 'xline_path_planner.*')),
)
