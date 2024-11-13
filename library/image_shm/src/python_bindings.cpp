#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "SharedImage.h"

namespace py = pybind11;

// Expose SharedImageWriter and SharedImageReader classes
PYBIND11_MODULE(shared_memory_image, m) {
    py::class_<SharedImageWriter>(m, "SharedImageWriter")
        .def(py::init<const std::string &>())
        .def("write_image", &SharedImageWriter::writeImage);

    py::class_<SharedImageReader>(m, "SharedImageReader")
        .def(py::init<const std::string &>())
        .def("read_image", &SharedImageReader::readImage);
}
