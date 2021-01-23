#include <System.h>
#include <ImuTypes.h>
#include "ndarray_converter.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
PYBIND11_MAKE_OPAQUE(std::vector<ORB_SLAM3::IMU::Point>)

void init_System(py::module &m){
    // need to explicity tell it how to deal with a vector of type ORBSLAM3::IMU::POINT
    py::bind_vector<std::vector<ORB_SLAM3::IMU::Point>>(m, "VectorPoint", py::module_local(true));
    NDArrayConverter::init_numpy();  // initialize ability to convert between numpy and cv::Mat

    py::class_<ORB_SLAM3::System>(m, "System")
            .def(py::init<const std::string &, const std::string &, const ORB_SLAM3::System::eSensor, const bool>(),
                py::arg("strVocFile"), py::arg("strSettingsFile"), py::arg("sensor"), py::arg("bUseViewer"),
                R"mydelimiter(
                Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.\n
                Parameters\n
                -----------\n
                string strVocFile: path to the vocabulary file.\n
                string strSettingsFile: path to the settings file which contains data like the camera parameters\n
                eSensor sensor: What mode to run the system in (Monocular, stereo, with imu, etc...)\n
                bool bUseViewer: Whether or not to activate the built-in GUI for ORBSLAM
                )mydelimiter")
            .def("Shutdown", &ORB_SLAM3::System::Shutdown,
                 R"mydelimiter(
                 All threads will be requested to finish. It waits until all threads have finished.\n
                 This function must be called before saving the trajectory.
                 )mydelimiter")
            .def("ActivateLocalizationMode", &ORB_SLAM3::System::ActivateLocalizationMode,
                 R"mydelimiter(
                        This stops local mapping thread (map building) and performs only camera tracking.
                         )mydelimiter")
            .def("TrackMonocular", &ORB_SLAM3::System::TrackMonocular,
                 py::arg("im"), py::arg("timestamp"), py::arg("vImuMeas")=std::vector<ORB_SLAM3::IMU::Point>(), py::arg("filename")="",
                 R"mydelimiter(
                 Proccess the given monocular frame and optionally imu data.\n
                 Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.\n
                 Returns the camera pose (empty if tracking fails).
                 )mydelimiter");

    py::enum_<ORB_SLAM3::System::eSensor>(m, "eSensor")  // This can be used from python with ORBSLAM3.eSensor.######
            .value("MONOCULAR", ORB_SLAM3::System::eSensor::MONOCULAR)
            .value("STEREO", ORB_SLAM3::System::eSensor::STEREO)
            .value("RGBD", ORB_SLAM3::System::eSensor::RGBD)
            .value("IMU_MONOCULAR", ORB_SLAM3::System::eSensor::IMU_MONOCULAR)
            .value("IMU_STEREO", ORB_SLAM3::System::eSensor::IMU_STEREO);
}