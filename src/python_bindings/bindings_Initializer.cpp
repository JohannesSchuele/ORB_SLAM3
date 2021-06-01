#include <Initializer.h>
#include "ndarray_converter.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

void init_Initializer(py::module &m){
    // need to explicity tell it how to deal with a vector of type ORBSLAM3::IMU::POINT
    NDArrayConverter::init_numpy();  // initialize ability to convert between numpy and cv::Mat

    py::class_<ORB_SLAM3::Initializer>(m, "Initializer")
            .def(py::init<const ORB_SLAM3::Frame &, float, int>(),
                py::arg("ReferenceFrame"), py::arg("sigma")=1.0, py::arg("iterations")=200,
                R"mydelimiter(
                Fix the reference frame
                )mydelimiter")
            .def("Initialize", &ORB_SLAM3::Initializer::Initialize,
                py::arg("CurrentFrame"), py::arg("Matches12"), py::arg("R21"), py::arg("t21"), py::arg("vP3D"), py::arg("vbTriangulated"),
                R"mydelimiter(
                 Computes in parallel a fundamental matrix and a homography
                 Selects a model and tries to recover the motion and the structure from motion
                 )mydelimiter");

}
