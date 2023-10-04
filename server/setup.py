from setuptools import setup, find_packages

setup(
    name='mocaprasp',
    version='1.0.0',
    packages=find_packages(),
    install_requires=[
        'click',
        'scipy',
        'numpy',
        'matplotlib',
        'opencv-python'
    ],
    entry_points="""
    [console_scripts]
    mocaprasp=mocaprasp:mocaprasp
    """
)