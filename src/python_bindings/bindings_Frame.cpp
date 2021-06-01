#include <Frame.h>
#include "ndarray_converter.h"
#include <ORBVocabulary.h>
#include <CameraModels/GeometricCamera.h>
#include <ORBextractor.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

void init_Frame(py::module &m){
    NDArrayConverter::init_numpy();  // initialize ability to convert between numpy and cv::Mat

    py::class_<ORB_SLAM3::Frame>(m, "Frame")
        //Frame(const Frame &frame);
        .def(py::init<const ORB_SLAM3::Frame &>(),
            py::arg("frame"),
            R"mydelimiter(
            Copy constructor.
            )mydelimiter")

/* includes virtual function
    //Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, 
    //cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());
        .def(py::init<const cv::Mat &, const cv::Mat &, const double &, ORB_SLAM3::ORBextractor, ORB_SLAM3::ORBextractor, ORB_SLAM3::ORBVocabulary, cv::Mat &, cv::Mat &,
        const float &, const float &, ORB_SLAM3::GeometricCamera, ORB_SLAM3::Frame, const ORB_SLAM3::IMU::Calib &>(),
            py::arg("imLeft"), py::arg("imRight"), py::arg("timeStamp"), py::arg("extractorLeft"), py::arg("extractorRight"), py::arg("voc"), py::arg("K"), py::arg("distCoef"),
            py::arg("bf"), py::arg("thDepth"), py::arg("pCamera"), py::arg("pPrevF"), py::arg("ImuCalib")=ORB_SLAM3::IMU::Calib(),
            R"mydelimiter(
            Constructor for stereo cameras.
            )mydelimiter")

    //Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, 
    //const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());
        .def(py::init<const cv::Mat &, const cv::Mat &, const double &, ORB_SLAM3::ORBextractor, ORB_SLAM3::ORBVocabulary, cv::Mat &, cv::Mat &,
        const float &, const float &, ORB_SLAM3::GeometricCamera, ORB_SLAM3::Frame, const ORB_SLAM3::IMU::Calib &>(),
            py::arg("imGray"), py::arg("imDepth"), py::arg("timeStamp"), py::arg("extractor"), py::arg("voc"), py::arg("K"), py::arg("distCoef"),
            py::arg("bf"), py::arg("thDepth"), py::arg("pCamera"), py::arg("pPrevF"), py::arg("ImuCalib")=ORB_SLAM3::IMU::Calib(),
            R"mydelimiter(
            Constructor for RGB-D cameras.
            )mydelimiter")

    //Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, 
    //const float &thDepth, Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());
        .def(py::init<const cv::Mat, const double &, ORB_SLAM3::ORBextractor, ORB_SLAM3::ORBVocabulary, ORB_SLAM3::GeometricCamera*, cv::Mat &, const float &, 
        const float &, ORB_SLAM3::Frame, const ORB_SLAM3::IMU::Calib &>(),
            py::arg("imGray"), py::arg("timeStamp"), py::arg("extractor"), py::arg("voc"), py::arg("pCamera"), py::arg("distCoef"), py::arg("bf"), 
            py::arg("thDepth"), py::arg("pPrevF"), py::arg("ImuCalib")=ORB_SLAM3::IMU::Calib(),
            R"mydelimiter(
            Constructor for Monocular cameras.
            )mydelimiter")
*/


        //void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);          
        .def("ExtractORB", &ORB_SLAM3::Frame::ExtractORB,
            py::arg("flag"), py::arg("im"), py::arg("x0"), py::arg("x1"),
            R"mydelimiter(
            Extract ORB on the image. 0 for left image and 1 for right image.
            )mydelimiter")

        //void ComputeBoW();
        .def("ComputeBow", &ORB_SLAM3::Frame::ComputeBoW,
            R"mydelimiter(
            Compute Bag of Words representation.
            )mydelimiter")

        //void SetPose(cv::Mat Tcw);
        .def("SetPose", &ORB_SLAM3::Frame::SetPose,
            py::arg("TcW"),
            R"mydelimiter(
            Set the camera pose. (Imu pose is not modified!)
            )mydelimiter")

        //void GetPose(cv::Mat &Tcw);
        .def("GetPose", &ORB_SLAM3::Frame::GetPose,
            py::arg("TcW"),
            R"mydelimiter(
            Set the camera pose. (Imu pose is not modified!)
            )mydelimiter")

        //void SetVelocity(const cv::Mat &Vwb);
        .def("SetVelocity", &ORB_SLAM3::Frame::SetVelocity,
            py::arg("Vwb"),
            R"mydelimiter(
            Set IMU velocity
            )mydelimiter")
    
        //void SetImuPoseVelocity(const cv::Mat &Rwb, const cv::Mat &twb, const cv::Mat &Vwb);
        .def("SetImuPoseVelocity", &ORB_SLAM3::Frame::SetImuPoseVelocity,
            py::arg("Rwb"), py::arg("twb"), py::arg("Vwb"),
            R"mydelimiter(
            Set IMU pose and velocity (implicitly changes camera pose)
            )mydelimiter")

        //void UpdatePoseMatrices();
        .def("UpdatePoseMatrices", &ORB_SLAM3::Frame::UpdatePoseMatrices,
            R"mydelimiter(
            Computes rotation, translation and camera center matrices from the camera pose.
            )mydelimiter")

        //inline cv::Mat GetCameraCenter()
        .def("GetCameraCenter", &ORB_SLAM3::Frame::GetCameraCenter,
            R"mydelimiter(
            Returns the camera center.
            )mydelimiter")
   
        //inline cv::Mat GetRotationInverse()
        .def("GetRotationInverse", &ORB_SLAM3::Frame::GetRotationInverse,
            R"mydelimiter(
            Returns inverse of rotation
            )mydelimiter");
           
                 
} 