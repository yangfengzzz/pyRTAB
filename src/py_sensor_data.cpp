//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/map.h>
#include <rtabmap/core/SensorData.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindSensorData(nb::module_ &m) {
    nb::class_<SensorData>(m, "SensorData")
            .def(nb::init<>())
            .def(nb::init<const cv::Mat &, int, double>())
            .def(nb::init<const cv::Mat &, const CameraModel &, int, double>())
            .def(nb::init<const cv::Mat &, const cv::Mat &, const CameraModel &, int, double>())
            // .def(nb::init<const cv::Mat &, const cv::Mat &, const cv::Mat &, const CameraModel &, int, double>())
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const CameraModel &, int, double>())
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const cv::Mat &, const CameraModel &,int, double>())
            .def(nb::init<const cv::Mat &, const cv::Mat &, const std::vector<CameraModel> &, int, double>())
            .def(nb::init<const cv::Mat &, const cv::Mat &, const cv::Mat &, const std::vector<CameraModel> &, int,double>())
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const std::vector<CameraModel> &, int,double>())
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const cv::Mat &,const std::vector<CameraModel> &, int, double>())
            .def(nb::init<const cv::Mat &, const cv::Mat &, const StereoCameraModel &, int, double>())
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const StereoCameraModel &, int,double>())
            .def(nb::init<const cv::Mat &, const cv::Mat &, const std::vector<StereoCameraModel> &, int, double>())
            .def(nb::init<const LaserScan &, const cv::Mat &, const cv::Mat &, const std::vector<StereoCameraModel> &,int, double>())
            .def(nb::init<const IMU &, int, double>())

            .def("isValid", &SensorData::isValid)
            .def("id", &SensorData::id)
            .def("setId", &SensorData::setId)
            .def("stamp", &SensorData::stamp)
            .def("setStamp", &SensorData::setStamp)
            .def("imageCompressed", &SensorData::imageCompressed)
            .def("depthOrRightCompressed", &SensorData::depthOrRightCompressed)
            .def("depthConfidenceCompressed", &SensorData::depthConfidenceCompressed)
            .def("laserScanCompressed", &SensorData::laserScanCompressed)
            .def("imageRaw", &SensorData::imageRaw)
            .def("depthOrRightRaw", &SensorData::depthOrRightRaw)
            .def("depthConfidenceRaw", &SensorData::depthConfidenceRaw)
            .def("laserScanRaw", &SensorData::laserScanRaw)
            .def("setRGBDImage", nb::overload_cast<const cv::Mat &, const cv::Mat &, const CameraModel &, bool>(
                                         &SensorData::setRGBDImage))
            .def("setRGBDImage", nb::overload_cast<const cv::Mat &, const cv::Mat &, const CameraModel &, bool>(
                                         &SensorData::setRGBDImage))
            .def("setRGBDImage",
                 nb::overload_cast<const cv::Mat &, const cv::Mat &, const cv::Mat &, const CameraModel &, bool>(
                         &SensorData::setRGBDImage))
            .def("setRGBDImage",
                 nb::overload_cast<const cv::Mat &, const cv::Mat &, const std::vector<CameraModel> &, bool>(
                         &SensorData::setRGBDImage))
            .def("setStereoImage", nb::overload_cast<const cv::Mat &, const cv::Mat &, const StereoCameraModel &, bool>(
                                           &SensorData::setStereoImage))
            .def("setStereoImage",
                 nb::overload_cast<const cv::Mat &, const cv::Mat &, const std::vector<StereoCameraModel> &, bool>(
                         &SensorData::setStereoImage))
            .def("setLaserScan", &SensorData::setLaserScan)
            .def("setCameraModel", &SensorData::setCameraModel)
            .def("setCameraModels", &SensorData::setCameraModels)
            .def("setStereoCameraModel", &SensorData::setStereoCameraModel)
            .def("setStereoCameraModels", &SensorData::setStereoCameraModels)
            .def("depthRaw", &SensorData::depthRaw)
            .def("rightRaw", &SensorData::rightRaw)
            .def("uncompressData", nb::overload_cast<>(&SensorData::uncompressData))

            .def("cameraModels", &SensorData::cameraModels)
            .def("stereoCameraModels", &SensorData::stereoCameraModels)
            .def("setUserData", &SensorData::setUserData)
            .def("userDataRaw", &SensorData::userDataRaw)
            .def("userDataCompressed", &SensorData::userDataCompressed)
            .def("setOccupancyGrid", &SensorData::setOccupancyGrid)
            .def("clearOccupancyGridRaw", &SensorData::clearOccupancyGridRaw)
            .def("gridGroundCellsRaw", &SensorData::gridGroundCellsRaw)
            .def("gridGroundCellsCompressed", &SensorData::gridGroundCellsCompressed)
            .def("gridObstacleCellsRaw", &SensorData::gridObstacleCellsRaw)
            .def("gridObstacleCellsCompressed", &SensorData::gridObstacleCellsCompressed)
            .def("gridEmptyCellsRaw", &SensorData::gridEmptyCellsRaw)
            .def("gridEmptyCellsCompressed", &SensorData::gridEmptyCellsCompressed)
            .def("gridCellSize", &SensorData::gridCellSize)
            .def("gridViewPoint", &SensorData::gridViewPoint)
            .def("setFeatures", &SensorData::setFeatures)
            .def("keypoints", &SensorData::keypoints)
            .def("keypoints3D", &SensorData::keypoints3D)
            .def("descriptors", &SensorData::descriptors)
            .def("addGlobalDescriptor", &SensorData::addGlobalDescriptor)
            .def("setGlobalDescriptors", &SensorData::setGlobalDescriptors)
            .def("clearGlobalDescriptors", &SensorData::clearGlobalDescriptors)
            .def("globalDescriptors", &SensorData::globalDescriptors)
            .def("setGroundTruth", &SensorData::setGroundTruth)
            .def("groundTruth", &SensorData::groundTruth)
            .def("setGlobalPose", &SensorData::setGlobalPose)
            .def("globalPose", &SensorData::globalPose)
            .def("globalPoseCovariance", &SensorData::globalPoseCovariance)
            .def("setGPS", &SensorData::setGPS)
            .def("gps", &SensorData::gps)
            .def("setIMU", &SensorData::setIMU)
            .def("imu", &SensorData::imu)
            .def("setEnvSensors", &SensorData::setEnvSensors)
            .def("addEnvSensor", &SensorData::addEnvSensor)
            .def("envSensors", &SensorData::envSensors)
            .def("setLandmarks", &SensorData::setLandmarks)
            .def("landmarks", &SensorData::landmarks)
            .def("getMemoryUsed", &SensorData::getMemoryUsed)
            .def("clearCompressedData", &SensorData::clearCompressedData)
            .def("clearRawData", &SensorData::clearRawData)
            .def("isPointVisibleFromCameras", &SensorData::isPointVisibleFromCameras)
            .def("envSensors", &SensorData::envSensors);
}