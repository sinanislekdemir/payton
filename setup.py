# from distutils.core import setup, find_packages
from setuptools import setup, find_packages

setup(
    name="Payton",
    version="0.0.0",
    author="sinan islekdemir",
    author_email="sinan@islekdemir.com",
    # Packages
    packages=find_packages(),
    # Include additional files into the package
    include_package_data=True,
    # Details
    url="https://27x2.com",
    #
    # license="LICENSE.txt",
    description="Payton.",
    # long_description=open("README.txt").read(),
    # Dependent packages (distributions)
    install_requires=[
        "numpy",
        "Pillow",
        "PyOpenGL",
        "pyrr",
        "PySDL2",
        "read",
    ],
)
