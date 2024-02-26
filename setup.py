from setuptools import setup

setup(
    name='ros_invariants_py',
    version='0.1.0',
    install_requires=[
        'numpy',
        'invariants_py',
    ],
    packages=['ros_invariants_py'],
    author='Maxim Vochten',
    author_email='maxim.vochten@kuleuven.be',
    description='ROS wrapper around invariants_py',
    license='MIT',
    tests_require=['pytest'],
)
