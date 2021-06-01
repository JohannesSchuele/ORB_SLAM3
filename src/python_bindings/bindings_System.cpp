#include <System.h>
#include <ImuTypes.h>
#include "ndarray_converter.h"
#include <MapPoint.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
PYBIND11_MAKE_OPAQUE(std::vector<ORB_SLAM3::IMU::Point>)

void init_System(py::module &m){
    // need to explicity tell it how to deal with a vector of type ORBSLAM3::IMU::POINT
    py::bind_vector<std::vector<ORB_SLAM3::IMU::Point>>(m, "VectorPoint", py::module_local(true));
    NDArrayConverter::init_numpy();  // initialize ability to convert between numpy and cv::Mat

    py::class_<ORB_SLAM3::Verbose>(m, "Verbose")

        //static eLevel th;

        //static void PrintMess(std::string str, eLevel lev)
        .def("PrintMess", 
                [] (std::string str, ORB_SLAM3::Verbose::eLevel lev) {return ORB_SLAM3::Verbose::PrintMess(str,lev);})
        //static void SetTh(eLevel _th)
        .def("SetTh",
            [] (ORB_SLAM3::Verbose::eLevel _th) {return ORB_SLAM3::Verbose::SetTh(_th);});

        py::enum_<ORB_SLAM3::Verbose::eLevel>(m, "eLevel")  
            .value("VERBOSITY_QUIET", ORB_SLAM3::Verbose::eLevel::VERBOSITY_QUIET)
            .value("VERBOSITY_NORMAL", ORB_SLAM3::Verbose::eLevel::VERBOSITY_NORMAL)
            .value("VERBOSITY_VERBOSE", ORB_SLAM3::Verbose::eLevel::VERBOSITY_VERBOSE)
            .value("VERBOSITY_VERY_VERBOSE", ORB_SLAM3::Verbose::eLevel::VERBOSITY_VERY_VERBOSE)
            .value("VERBOSITY_DEBUG", ORB_SLAM3::Verbose::eLevel::VERBOSITY_DEBUG);


//System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true
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
            .def("TrackStereo", &ORB_SLAM3::System::TrackStereo,
                py::arg("imLeft"), py::arg("imRight"), py::arg("timestamp"), py::arg("vImuMeas")=std::vector<ORB_SLAM3::IMU::Point>(), py::arg("filename")="",
                R"mydelimiter(
                 Proccess the given stereo frame. Images must be synchronized and rectified.
                 Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
                 Returns the camera pose (empty if tracking fails).
                 )mydelimiter")
            .def("TrackRGBD", &ORB_SLAM3::System::TrackRGBD,
                py::arg("im"), py::arg("depthmap"), py::arg("timestamp"), py::arg("filename")="",
                R"mydelimiter(
                Process the given rgbd frame. Depthmap must be registered to the RGB frame.
                Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
                Input depthmap: Float (CV_32F).
                Returns the camera pose (empty if tracking fails).
                 )mydelimiter")
            .def("TrackMonocular", &ORB_SLAM3::System::TrackMonocular,
                py::arg("im"), py::arg("timestamp"), py::arg("vImuMeas")=std::vector<ORB_SLAM3::IMU::Point>(), py::arg("filename")="",
                R"mydelimiter(
                Proccess the given monocular frame and optionally imu data.\n
                Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.\n
                Returns the camera pose (empty if tracking fails).
                )mydelimiter")
            .def("ActivateLocalizationMode", &ORB_SLAM3::System::ActivateLocalizationMode,
                R"mydelimiter(
                This stops local mapping thread (map building) and performs only camera tracking.
                )mydelimiter")
            .def("DeactivateLocalizationMode", &ORB_SLAM3::System::DeactivateLocalizationMode,
                R"mydelimiter(
                This resumes local mapping thread and performs SLAM again.
                )mydelimiter")
            .def("MapChanged", &ORB_SLAM3::System::MapChanged,
                R"mydelimiter(
                Returns true if there have been a big map change (loop closure, global BA)
                since last call to this function
                )mydelimiter")  
            .def("Reset", &ORB_SLAM3::System::Reset,
                R"mydelimiter(
                Reset the system (clear Atlas or the active map)
                )mydelimiter")    
            .def("ResetActiveMap", &ORB_SLAM3::System::ResetActiveMap)
            .def("Shutdown", &ORB_SLAM3::System::Shutdown,
                R"mydelimiter(
                All threads will be requested to finish. It waits until all threads have finished.\n
                This function must be called before saving the trajectory.
                )mydelimiter")
            .def("SaveTrajectoryTUM", &ORB_SLAM3::System::SaveTrajectoryTUM,
                py::arg("filename"), 
                R"mydelimiter(
                Save camera trajectory in the TUM RGB-D dataset format.
                Only for stereo and RGB-D. This method does not work for monocular.
                Call first Shutdown()
                See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
                )mydelimiter")    
            .def("SaveKeyFrameTrajectoryTUM", &ORB_SLAM3::System::SaveKeyFrameTrajectoryTUM,
                py::arg("filename"), 
                R"mydelimiter(
                Save keyframe poses in the TUM RGB-D dataset format.
                This method works for all sensor input.
                Call first Shutdown()
                See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
                )mydelimiter")                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
            .def("SaveTrajectoryEuRoC", &ORB_SLAM3::System::SaveTrajectoryEuRoC,
                py::arg("filename"))
            .def("SaveKeyFrameTrajectoryEuRoC", &ORB_SLAM3::System::SaveKeyFrameTrajectoryEuRoC,
                py::arg("filename"))    
            .def("SaveDebugData", &ORB_SLAM3::System::SaveDebugData,
                py::arg("iniIdx"), 
                R"mydelimiter(
                Save data used for initialization debug
                )mydelimiter")     
            .def("SaveTrajectoryKITTI", &ORB_SLAM3::System::SaveTrajectoryKITTI,
                py::arg("iniIdx"), 
                R"mydelimiter(
                Save camera trajectory in the KITTI dataset format.
                Only for stereo and RGB-D. This method does not work for monocular.
                Call first Shutdown()
                See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
                )mydelimiter")         
            // TODO: Save/Load functions
            // SaveMap(const string &filename);
            // LoadMap(const string &filename);
            .def("GetTrackingState", &ORB_SLAM3::System::GetTrackingState, 
                R"mydelimiter(
                Information from most recent processed frame
                You can call this right after TrackMonocular (or stereo or RGBD)
                )mydelimiter")

            //std::vector<MapPoint*> GetTrackedMapPoints();
            .def("GetTrackedMapPoints", &ORB_SLAM3::System::GetTrackedMapPoints, 
                R"mydelimiter(
                Information from most recent processed frame
                You can call this right after TrackMonocular (or stereo or RGBD)
                )mydelimiter")

            //std::vector<cv::KeyPoint> GetTrackedKeyPointsUn(); 
            .def("GetTrackedKeyPointsUn", &ORB_SLAM3::System::GetTrackedKeyPointsUn, 
                R"mydelimiter(
                Information from most recent processed frame
                You can call this right after TrackMonocular (or stereo or RGBD)
                )mydelimiter")

            // For debugging
            .def("GetTimeFromIMUInit", &ORB_SLAM3::System::GetTimeFromIMUInit)
            .def("isLost", &ORB_SLAM3::System::isLost)
            .def("isFinished", &ORB_SLAM3::System::isFinished)
            .def("ChangeDataset", &ORB_SLAM3::System::ChangeDataset);

            //.def("SaveAtlas", &ORB_SLAM3::System::SaveAtlas
            //    py::arg("type"));
  
   
    py::enum_<ORB_SLAM3::System::eSensor>(m, "eSensor")  // This can be used from python with ORBSLAM3.eSensor.######
            .value("MONOCULAR", ORB_SLAM3::System::eSensor::MONOCULAR)
            .value("STEREO", ORB_SLAM3::System::eSensor::STEREO)
            .value("RGBD", ORB_SLAM3::System::eSensor::RGBD)
            .value("IMU_MONOCULAR", ORB_SLAM3::System::eSensor::IMU_MONOCULAR)
            .value("IMU_STEREO", ORB_SLAM3::System::eSensor::IMU_STEREO);

    py::enum_<ORB_SLAM3::System::eFileType>(m, "eFileType")
            .value("TEXT_FILE", ORB_SLAM3::System::eFileType::TEXT_FILE)
            .value("BINARY_FILE", ORB_SLAM3::System::eFileType::BINARY_FILE);


}