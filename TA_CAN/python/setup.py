from setuptools import setup, find_packages
import platform

setup(
    name='motor_control',
    version='1.0',
    packages=['motor_control'],
    package_dir={'': 'src'},
    license='mwj 1.0',
    description='A Python library for motor control',
    author='MaWeiJie',
    author_email='maweijie@jacode.cn',
    install_requires=['pyserial']
)
