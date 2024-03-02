from setuptools import setup

setup(
    name='invariants_py_ros',
    version='0.1.0',
    install_requires=[
        'numpy',
        'invariants_py',
    ],
    packages=['invariants_py_ros'],
    author='Maxim Vochten',
    author_email='maxim.vochten@kuleuven.be',
    description='ROS wrapper around invariants_py',
    license='MIT',
    tests_require=['pytest'],
)
