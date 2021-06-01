#include <Atlas.h>
#include <Viewer.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
void init_Atlas(py::module &m){

    //template<class Archive>
    //void serialize(Archive &ar, const unsigned int version)
    //.def("serialize", &ORB_SLAM3::Atlas::serialize,
    //    py::arg("ar"), py::arg("version"))

    //Atlas(int initKFid);
    py::class_<ORB_SLAM3::Atlas>(m, "Atlas")
        .def(py::init<int>(),
            py::arg("initKFid"),
            R"mydelimiter(
            When its initialization the first map is created
            )mydelimiter")
        .def("CreateNewMap", &ORB_SLAM3::Atlas::CreateNewMap)
        .def("ChangeMap", &ORB_SLAM3::Atlas::ChangeMap,
            py::arg("pMap"))
        .def("GetLastInitKFid", &ORB_SLAM3::Atlas::GetLastInitKFid)
        .def("SetViewer", &ORB_SLAM3::Atlas::SetViewer,
            py::arg("pViewer"))
        .def("AddKeyFrame", &ORB_SLAM3::Atlas::AddKeyFrame,
            py::arg("pF"),
            R"mydelimiter(
            Method for change components in the current map
            )mydelimiter")
        .def("AddMapPoint", &ORB_SLAM3::Atlas::AddMapPoint,
            py::arg("pF"),
            R"mydelimiter(
            Method for change components in the current map
            )mydelimiter")
        //void EraseMapPoint(MapPoint* pMP);
        //void EraseKeyFrame(KeyFrame* pKF);
        .def("AddCamera", &ORB_SLAM3::Atlas::AddCamera,
            py::arg("pCam"))
        /* All methods without Map pointer work on current map */
        .def("SetReferenceMapPoints", &ORB_SLAM3::Atlas::SetReferenceMapPoints,
            py::arg("vpMPs"))
        .def("InformNewBigChange", &ORB_SLAM3::Atlas::InformNewBigChange)
        .def("GetLastBigChangeIdx", &ORB_SLAM3::Atlas::GetLastBigChangeIdx)
        .def("MapPointsInMap", &ORB_SLAM3::Atlas::MapPointsInMap)
        .def("KeyFramesInMap", &ORB_SLAM3::Atlas::KeyFramesInMap)
        // Method for get data in current map
        .def("GetAllKeyFrames", &ORB_SLAM3::Atlas::GetAllKeyFrames)
        .def("GetAllMapPoints", &ORB_SLAM3::Atlas::GetAllMapPoints)
        .def("GetReferenceMapPoints", &ORB_SLAM3::Atlas::GetReferenceMapPoints)
        .def("GetAllMaps", &ORB_SLAM3::Atlas::GetAllMaps)
        .def("CountMaps", &ORB_SLAM3::Atlas::CountMaps)
        .def("clearMap", &ORB_SLAM3::Atlas::clearMap)
        .def("clearAtlas", &ORB_SLAM3::Atlas::clearAtlas)
        .def("SetMapBad", &ORB_SLAM3::Atlas::SetMapBad,
            py::arg("pMap"))
        .def("RemoveBadMaps", &ORB_SLAM3::Atlas::RemoveBadMaps)
        .def("isInertial", &ORB_SLAM3::Atlas::isInertial)
        .def("SetInertialSensor", &ORB_SLAM3::Atlas::SetInertialSensor)
        .def("SetImuInitialized", &ORB_SLAM3::Atlas::SetImuInitialized)
        .def("isImuInitialized", &ORB_SLAM3::Atlas::isImuInitialized)
        // Function for garantee the correction of serialization of this object
        .def("PreSave", &ORB_SLAM3::Atlas::PreSave)
        .def("PostLoad", &ORB_SLAM3::Atlas::PostLoad)
        .def("SetKeyFrameDababase", &ORB_SLAM3::Atlas::SetKeyFrameDababase,
            py::arg("pKFDB"))
        .def("GetKeyFrameDatabase", &ORB_SLAM3::Atlas::GetKeyFrameDatabase)
        .def("SetORBVocabulary", &ORB_SLAM3::Atlas::SetORBVocabulary,
            py::arg("pORBVoc"))
        .def("GetNumLivedKF", &ORB_SLAM3::Atlas::GetNumLivedKF)
        .def("GetNumLivedMP", &ORB_SLAM3::Atlas::GetNumLivedMP);
}