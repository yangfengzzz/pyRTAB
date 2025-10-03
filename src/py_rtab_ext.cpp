//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/list.h>
#include <nanobind/stl/pair.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Memory.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace rtabmap;

using namespace nb::literals;

void bindOdometry(nb::module_ &m);
void bindTransform(nb::module_ &m);
void bindCameraModel(nb::module_ &m);
void bindSensorData(nb::module_ &m);
void bindLaserScan(nb::module_ &m);
void bindStereoCameraModel(nb::module_ &m);
void bindStatistics(nb::module_ &m);
void bindSignature(nb::module_ &m);
void bindLink(nb::module_ &m);
void bindMemory(nb::module_ &m);
void bindFeature2D(nb::module_ &m);
void bindVWDictionary(nb::module_ &m);
void bindVisualWord(nb::module_ &m);

void bindSensorCapture(nb::module_ &m);
void bindCamera(nb::module_ &m);
void bindCameraImages(nb::module_& m);
void bindCameraRGBD(nb::module_& m);

NB_MODULE(rtab_ext, m) {
    m.doc() = "nanobind for rtabmap";

    bindOdometry(m);
    bindTransform(m);
    bindCameraModel(m);
    bindSensorData(m);
    bindLaserScan(m);
    bindStereoCameraModel(m);
    bindStatistics(m);
    bindSignature(m);
    bindLink(m);
    bindMemory(m);
    bindFeature2D(m);
    bindVWDictionary(m);
    bindVisualWord(m);

    bindSensorCapture(m);
    bindCamera(m);
    bindCameraImages(m);
    bindCameraRGBD(m);

    nb::class_<Rtabmap>(m, "Rtabmap")
            .def(nb::init<>())
            .def("process",
                 nb::overload_cast<const SensorData &, Transform, const cv::Mat &, const std::vector<float> &,
                                   const std::map<std::string, float> &>(&Rtabmap::process),
                 "data"_a, "odomPose"_a, "odomCovariance"_a, "odomVelocity"_a, "externalStats"_a)
            .def("process",
                 nb::overload_cast<const SensorData &, Transform, float, float, const std::vector<float> &,
                                   const std::map<std::string, float> &>(&Rtabmap::process),
                 "data"_a, "odomPose"_a, "odomLinearVariance"_a, "odomAngularVariance"_a, "odomVelocity"_a,
                 "externalStats"_a)
            .def("process",
                 nb::overload_cast<const cv::Mat &, int, const std::map<std::string, float> &>(&Rtabmap::process),
                 "image"_a, "id"_a, "externalStats"_a)
            .def("init", nb::overload_cast<const ParametersMap &, const std::string &, bool>(&Rtabmap::init),
                 "parameters"_a, "databasePath"_a = "", "loadDatabaseParameters"_a = false)
            .def("init", nb::overload_cast<const std::string &, const std::string &, bool>(&Rtabmap::init),
                 "configFile"_a = "", "databasePath"_a = "", "loadDatabaseParameters"_a = false)
            .def("close", &Rtabmap::close, "databaseSaved"_a = true, "ouputDatabasePath"_a = "")
            .def("getWorkingDir", &Rtabmap::getWorkingDir)
            .def("isRGBDMode", &Rtabmap::isRGBDMode)
            .def("getLoopClosureId", &Rtabmap::getLoopClosureId)
            .def("getLoopClosureValue", &Rtabmap::getLoopClosureValue)
            .def("getHighestHypothesisId", &Rtabmap::getHighestHypothesisId)
            .def("getHighestHypothesisValue", &Rtabmap::getHighestHypothesisValue)
            .def("getLastLocationId", &Rtabmap::getLastLocationId)
            .def("getWM", &Rtabmap::getWM)
            .def("getSTM", &Rtabmap::getSTM)
            .def("getWMSize", &Rtabmap::getWMSize)
            .def("getSTMSize", &Rtabmap::getSTMSize)
            .def("getWeights", &Rtabmap::getWeights)
            .def("getTotalMemSize", &Rtabmap::getTotalMemSize)
            .def("getLastProcessTime", &Rtabmap::getLastProcessTime)
            .def("isInSTM", &Rtabmap::isInSTM, "locationId"_a)
            .def("isIDsGenerated", &Rtabmap::isIDsGenerated)
            .def("getStatistics", &Rtabmap::getStatistics)
            .def("getLocalOptimizedPoses", &Rtabmap::getLocalOptimizedPoses)
            // .def("getLocalConstraints", &Rtabmap::getLocalConstraints)
            .def("getPose", &Rtabmap::getPose, "locationId"_a)
            .def("getMapCorrection", &Rtabmap::getMapCorrection)
            .def("getMemory", &Rtabmap::getMemory)
            .def("getGoalReachedRadius", &Rtabmap::getGoalReachedRadius)
            .def("getLocalRadius", &Rtabmap::getLocalRadius)
            .def("getLastLocalizationPose", &Rtabmap::getLastLocalizationPose)
            .def("getTimeThreshold", &Rtabmap::getTimeThreshold)
            .def("setTimeThreshold", &Rtabmap::setTimeThreshold, "maxTimeAllowed"_a)
            .def("getMemoryThreshold", &Rtabmap::getMemoryThreshold)
            .def("setMemoryThreshold", &Rtabmap::setMemoryThreshold, "maxMemoryAllowed"_a)
            .def("setInitialPose", &Rtabmap::setInitialPose, "initialPose"_a)
            .def("triggerNewMap", &Rtabmap::triggerNewMap)
            .def("labelLocation", &Rtabmap::labelLocation, "id"_a, "label"_a)
            .def("setUserData", &Rtabmap::setUserData, "id"_a, "data"_a)
            .def("generateDOTGraph", &Rtabmap::generateDOTGraph, "path"_a, "id"_a = 0, "margin"_a = 5)
            .def("exportPoses", &Rtabmap::exportPoses, "path"_a, "optimized"_a, "globals"_a, "format"_a)
            .def("resetMemory", &Rtabmap::resetMemory)
            .def("dumpPrediction", &Rtabmap::dumpPrediction)
            .def("dumpData", &Rtabmap::dumpData)
            .def("parseParameters", &Rtabmap::parseParameters, "parameters"_a)
            .def("getParameters", &Rtabmap::getParameters)
            .def("setWorkingDirectory", &Rtabmap::setWorkingDirectory, "path"_a)
            .def("rejectLastLoopClosure", &Rtabmap::rejectLastLoopClosure)
            .def("deleteLastLocation", &Rtabmap::deleteLastLocation)
            // .def("setOptimizedPoses", &Rtabmap::setOptimizedPoses)
            .def("getSignatureCopy", &Rtabmap::getSignatureCopy, "id"_a, "images"_a, "scan"_a, "userData"_a,
                 "occupancyGrid"_a, "withWords"_a, "withGlobalDescriptors"_a)
            // .def("getGraph", &Rtabmap::getGraph)
            .def("getNodesInRadius",
                 nb::overload_cast<const Transform &, float, int, std::map<int, float> *>(&Rtabmap::getNodesInRadius),
                 "pose"_a, "radius"_a, "k"_a, "distsSqr"_a)
            .def("getNodesInRadius",
                 nb::overload_cast<int, float, int, std::map<int, float> *>(&Rtabmap::getNodesInRadius), "nodeId"_a,
                 "radius"_a, "k"_a, "distsSqr"_a)
            .def("detectMoreLoopClosures", &Rtabmap::detectMoreLoopClosures, "clusterRadiusMax"_a = 0.5f,
                 "clusterAngle"_a = M_PI / 6.0f, "iterations"_a = 1, "intraSession"_a = true, "interSession"_a = true,
                 "state"_a = 0, "clusterRadiusMin"_a = 0.0f)
            .def("globalBundleAdjustment", &Rtabmap::globalBundleAdjustment, "optimizerType"_a = 1 /*g2o*/,
                 "rematchFeatures"_a = true, "iterations"_a = 0, "pixelVariance"_a = 0.0f)
            .def("cleanupLocalGrids", &Rtabmap::cleanupLocalGrids, "mapPoses"_a, "map"_a, "xMin"_a, "yMin"_a,
                 "cellSize"_a, "cropRadius"_a = 1, "filterScans"_a = false)
            .def("refineLinks", &Rtabmap::refineLinks)
            .def("addLink", &Rtabmap::addLink, "link"_a)
            .def("getInformation", &Rtabmap::getInformation, "covariance"_a)
            .def("addNodesToRepublish", &Rtabmap::addNodesToRepublish, "ids"_a)
            .def("getPathStatus", &Rtabmap::getPathStatus)
            .def("clearPath", &Rtabmap::clearPath, "status"_a)
            .def("computePath", nb::overload_cast<int, bool>(&Rtabmap::computePath), "targetNode"_a, "globals"_a)
            .def("computePath", nb::overload_cast<const Transform &, float>(&Rtabmap::computePath), "targetPose"_a,
                 "tolerance"_a)
            .def("getPath", &Rtabmap::getPath)
            .def("getPathNextPoses", &Rtabmap::getPathNextPoses)
            .def("getPathNextNodes", &Rtabmap::getPathNextNodes)
            .def("getPathCurrentGoalId", &Rtabmap::getPathCurrentGoalId)
            .def("getPathCurrentIndex", &Rtabmap::getPathCurrentIndex)
            .def("getPathCurrentGoalIndex", &Rtabmap::getPathCurrentGoalIndex)
            .def("getPathTransformToGoal", &Rtabmap::getPathTransformToGoal)
            .def("getForwardWMPoses", &Rtabmap::getForwardWMPoses, "fromId"_a, "maxNearestNeighbors"_a, "radius"_a,
                 "maxDiffID"_a)
            .def("getPaths", &Rtabmap::getPaths, "poses"_a, "target"_a, "maxGraphDepth"_a = 0)
            .def("adjustLikelihood", &Rtabmap::adjustLikelihood, "likelihood"_a);
    // .def("selectHypothesis", &Rtabmap::selectHypothesis);

    nb::class_<ProgressState>(m, "ProgressState")
            .def(nb::init<>())
            .def("setCanceled", &ProgressState::setCanceled, "canceled"_a)
            .def("isCanceled", &ProgressState::isCanceled);
}