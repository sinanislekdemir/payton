# from distutils.core import setup, find_packages
from setuptools import setup, find_packages

setup(
    name="Payton",
    version="0.0.2.11",
    author="Sinan ISLEKDEMIR",
    author_email="sinan@islekdemir.com",
    # Packages
    packages=find_packages(),
    # Include additional files into the package
    include_package_data=True,
    # Details
    url="https://github.com/sinanislekdemir/payton",
    #
    license="GNU GPLv3",
    description="Payton 3D Kickstart Toolkit",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    # Dependent packages (distributions)
    install_requires=["numpy", "Pillow", "PyOpenGL", "pyrr", "PySDL2", "read"],
)
