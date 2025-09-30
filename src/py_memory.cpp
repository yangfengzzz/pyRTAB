//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/list.h>
#include <nanobind/stl/set.h>
#include <nanobind/eigen/dense.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/RegistrationInfo.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindMemory(nb::module_& m) {
    nb::class_<Memory>(m, "Memory")
            .def(nb::init<>())
            .def("parseParameters", &Memory::parseParameters)
            .def("getParameters", &Memory::getParameters)
            .def("update", nb::overload_cast<const SensorData&, Statistics*>(&Memory::update), "data"_a, "stats"_a)
            .def("update",
                 nb::overload_cast<const SensorData&, const Transform&, const cv::Mat&, const std::vector<float>&,
                                   Statistics*>(&Memory::update),
                 "data"_a, "pose"_a, "covariance"_a, "velocity"_a, "stats"_a)
            .def("init", &Memory::init)
            .def("close", &Memory::close)
            .def("computeLikelihood", &Memory::computeLikelihood)
            .def("incrementMapId", &Memory::incrementMapId)
            .def("updateAge", &Memory::updateAge)
            .def("forget", &Memory::forget)
            .def("reactivateSignatures", &Memory::reactivateSignatures)
            .def("cleanup", &Memory::cleanup)
            .def("saveStatistics", &Memory::saveStatistics)
            .def("savePreviewImage", &Memory::savePreviewImage)
            .def("loadPreviewImage", &Memory::loadPreviewImage)
            .def("saveOptimizedPoses", &Memory::saveOptimizedPoses)
            .def("loadOptimizedPoses", &Memory::loadOptimizedPoses)
            .def("save2DMap", &Memory::save2DMap)
            .def("load2DMap", &Memory::load2DMap)
            .def("saveOptimizedMesh", &Memory::saveOptimizedMesh)
            .def("loadOptimizedMesh", &Memory::loadOptimizedMesh)
            .def("emptyTrash", &Memory::emptyTrash)
            .def("joinTrashThread", &Memory::joinTrashThread)
            .def("addLink", &Memory::addLink)
            .def("updateLink", &Memory::updateLink)
            .def("removeAllVirtualLinks", &Memory::removeAllVirtualLinks)
            .def("removeVirtualLinks", &Memory::removeVirtualLinks)
            .def("getNeighborsId", &Memory::getNeighborsId)
            .def("getNeighborsIdRadius", &Memory::getNeighborsIdRadius)
            .def("deleteLocation", &Memory::deleteLocation)
            .def("saveLocationData", &Memory::saveLocationData)
            .def("removeLink", &Memory::removeLink)
            .def("removeRawData", &Memory::removeRawData)
            .def("getWorkingMem", &Memory::getWorkingMem)
            .def("getStMem", &Memory::getStMem)
            .def("getMaxStMemSize", &Memory::getMaxStMemSize)
            // .def("getNeighborLinks", &Memory::getNeighborLinks)
            // .def("getLoopClosureLinks", &Memory::getLoopClosureLinks)
            // .def("getLinks", &Memory::getLinks)
            // .def("getAllLinks", &Memory::getAllLinks)
            .def("isBinDataKept", &Memory::isBinDataKept)
            .def("getSimilarityThreshold", &Memory::getSimilarityThreshold)
            .def("getWeights", &Memory::getWeights)
            .def("getLastSignatureId", &Memory::getLastSignatureId)
            .def("getLastWorkingSignature", &Memory::getLastWorkingSignature)
            .def("getNodesObservingLandmark", &Memory::getNodesObservingLandmark)
            .def("getSignatureIdByLabel", &Memory::getSignatureIdByLabel)
            .def("labelSignature", &Memory::labelSignature)
            .def("getAllLabels", &Memory::getAllLabels)
            .def("getLandmarksIndex", &Memory::getLandmarksIndex)
            .def("allNodesInWM", &Memory::allNodesInWM)

            .def("setUserData", &Memory::setUserData)
            .def("getDatabaseMemoryUsed", &Memory::getDatabaseMemoryUsed)
            .def("getDatabaseVersion", &Memory::getDatabaseVersion)
            .def("getDatabaseUrl", &Memory::getDatabaseUrl)
            .def("getDbSavingTime", &Memory::getDbSavingTime)
            .def("getMapId", &Memory::getMapId)
            .def("getOdomPose", &Memory::getOdomPose)
            .def("getGroundTruthPose", &Memory::getGroundTruthPose)
            .def("getGroundTruths", &Memory::getGroundTruths)
            .def("getGPS", &Memory::getGPS)
            .def("getNodeInfo", &Memory::getNodeInfo)
            .def("getImageCompressed", &Memory::getImageCompressed)
            .def("getNodeData", &Memory::getNodeData)
            // .def("getNodeWordsAndGlobalDescriptors", &Memory::getNodeWordsAndGlobalDescriptors)
            .def("getNodeCalibration", &Memory::getNodeCalibration)
            .def("getAllSignatureIds", &Memory::getAllSignatureIds)
            .def("memoryChanged", &Memory::memoryChanged)
            .def("isIncremental", &Memory::isIncremental)
            .def("isLocalizationDataSaved", &Memory::isLocalizationDataSaved)
            .def("getSignature", &Memory::getSignature)
            .def("isInSTM", &Memory::isInSTM)
            .def("isInWM", &Memory::isInWM)
            .def("isInLTM", &Memory::isInLTM)
            .def("isIDsGenerated", &Memory::isIDsGenerated)
            .def("getLastGlobalLoopClosureId", &Memory::getLastGlobalLoopClosureId)
            .def("getFeature2D", &Memory::getFeature2D)
            .def("isGraphReduced", &Memory::isGraphReduced)
            .def("getOdomMaxInf", &Memory::getOdomMaxInf)
            .def("isOdomGravityUsed", &Memory::isOdomGravityUsed)
            .def("dumpMemoryTree", &Memory::dumpMemoryTree)
            .def("dumpMemory", &Memory::dumpMemory)
            .def("dumpSignatures", &Memory::dumpSignatures)
            .def("dumpDictionary", &Memory::dumpDictionary)
            .def("getMemoryUsed", &Memory::getMemoryUsed)
            .def("generateGraph", &Memory::generateGraph)
            .def("cleanupLocalGrids", &Memory::cleanupLocalGrids)
            .def("getVWDictionary", &Memory::getVWDictionary)
            // .def("getMetricConstraints", &Memory::getMetricConstraints)
            .def("computeTransform",
                 nb::overload_cast<Signature&, Signature&, Transform, RegistrationInfo*, bool>(
                         &Memory::computeTransform, nb::const_),
                 "fromS"_a, "toS"_a, "guess"_a, "info"_a, "useKnownCorrespondencesIfPossible"_a)
            .def("computeTransform",
                 nb::overload_cast<int, int, Transform, RegistrationInfo*, bool>(&Memory::computeTransform), "fromId"_a,
                 "toId"_a, "guess"_a, "info"_a, "useKnownCorrespondencesIfPossible"_a)
            .def("computeIcpTransform", &Memory::computeIcpTransform)
            .def("computeIcpTransformMulti", &Memory::computeIcpTransformMulti);
}