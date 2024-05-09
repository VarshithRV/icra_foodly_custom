from setuptools import find_packages
from setuptools import setup

setup(
    name='foodly_rd_description',
    version='0.1.0',
    packages=find_packages(
        include=('foodly_rd_description', 'foodly_rd_description.*')),
)
