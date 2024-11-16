from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import pybind11
import os

class get_pybind_include(object):
    """Helper class to determine the pybind11 include path"""
    def __str__(self):
        return pybind11.get_include()

# Set OpenCV include path manually if built from source
opencv_include_dir = "/usr/local/include/opencv4"  # Adjust this path as necessary

# Define the Python extension module
ext_modules = [
    Extension(
        "shared_memory_image",
        ["src/python_bindings.cpp", "src/SharedImageWriter.cpp", "src/SharedImageReader.cpp"],
        include_dirs=[
            "include",
            get_pybind_include(),
            opencv_include_dir,
        ],
        libraries=["opencv_core", "opencv_imgcodecs", "opencv_highgui"],
        language="c++"
    ),
]

# Define the setup configuration
setup(
    name="shared_memory_image",
    version="1.0",
    author="Your Name",
    description="Python bindings for shared memory image transport",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    install_requires=["pybind11>=2.5.0"],
)



