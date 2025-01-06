from setuptools import setup, find_packages

setup(
    name='gym_pybullet_drones',
    version='1.0',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'pybullet',
        'gymnasium',
        'matplotlib',
        'scipy',
        'stable-baselines3',
        'transforms3d'
    ],
    author='Your Name',
    description='Gym environments for drones using PyBullet',
)
