#include <FrameDrawer.h>
#include <Atlas.h>
#include "ndarray_converter.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
void init_FrameDrawer(py::module &m){
    NDArrayConverter::init_numpy();  // initialize ability to convert between numpy and cv::Mat

    py::class_<ORB_SLAM3::FrameDrawer>(m, "FrameDrawer")
        //FrameDrawer(Atlas* pAtlas);
        .def(py::init<ORB_SLAM3::Atlas*>(),
            py::arg("pAtlas"))

        .def("Update", &ORB_SLAM3::FrameDrawer::Update,
            py::arg("pTracker"),
            R"mydelimiter(
            Update info from the last processed frame.
            )mydelimiter")
        .def("DrawFrame", &ORB_SLAM3::FrameDrawer::DrawFrame,
            py::arg("bOldFeatures")=true,
            R"mydelimiter(
            Draw last processed frame.
            )mydelimiter")
        .def("DrawRightFrame", &ORB_SLAM3::FrameDrawer::DrawRightFrame)

        .def_readwrite("both", &ORB_SLAM3::FrameDrawer::both);

}