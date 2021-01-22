#include <System.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(ORBSLAM3, m){
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
            .def("ActivateLocalizationMode", &ORB_SLAM3::System::ActivateLocalizationMode,
                 R"mydelimiter(
                        This stops local mapping thread (map building) and performs only camera tracking.
                         )mydelimiter");

    py::enum_<ORB_SLAM3::System::eSensor>(m, "eSensor")  // This can be used from python with ORBSLAM3.eSensor.######
            .value("MONOCULAR", ORB_SLAM3::System::eSensor::MONOCULAR)
            .value("STEREO", ORB_SLAM3::System::eSensor::STEREO)
            .value("RGBD", ORB_SLAM3::System::eSensor::RGBD)
            .value("IMU_MONOCULAR", ORB_SLAM3::System::eSensor::IMU_MONOCULAR)
            .value("IMU_STEREO", ORB_SLAM3::System::eSensor::IMU_STEREO);
}