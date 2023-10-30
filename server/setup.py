from setuptools import setup, find_packages

setup(
    name='mocaprasp',
    version='1.0.0',
    packages=find_packages(),
    install_requires=[
        'opencv-python', 
        'numpy',
        'scipy',
        'scikit-learn',
        'matplotlib',
        'click'
    ],
    entry_points="""
    [console_scripts]
    mocaprasp=mocaprasp:mocaprasp
    """
)