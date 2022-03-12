# from distutils.core import setup, find_packages
import sys

from setuptools import find_packages, setup

# For old pip versions
if sys.version_info < (3, 8):
    sys.exit("Sorry, Python < 3.8 is not supported")

setup(
    name="Payton",
    version="v1.0.3",
    author="Sinan ISLEKDEMIR",
    author_email="sinan@islekdemir.com",
    # Packages
    packages=find_packages(),
    # Include additional files into the package
    include_package_data=True,
    # Details
    url="https://github.com/sinanislekdemir/payton",
    #
    license="BSD License",
    description="Payton 3D Kickstart Toolkit",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    python_requires=">3.8",
    # Dependent packages (distributions)
    install_requires=[
        "Pillow>=9.0.1",
        "PyOpenGL>=3.1.6",
        "pyrr>=0.10.3",
        "PySDL2>=0.9.11",
        "numpy>=1.22.3",
        "Cython>=0.29.28",
    ],
)
