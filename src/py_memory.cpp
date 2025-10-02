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
            .def("parseParameters", &Memory::parseParameters, "parameters"_a)
            .def("getParameters", &Memory::getParameters)
            .def("update", nb::overload_cast<const SensorData&, Statistics*>(&Memory::update), "data"_a, "stats"_a)
            .def("update",
                 nb::overload_cast<const SensorData&, const Transform&, const cv::Mat&, const std::vector<float>&,
                                   Statistics*>(&Memory::update),
                 "data"_a, "pose"_a, "covariance"_a, "velocity"_a, "stats"_a)
            .def("init", &Memory::init, "dbUrl"_a, "dbOverwritten"_a, "parameters"_a, "postInitClosingEvents"_a)
            .def("close", &Memory::close, "databaseSaved"_a = true, "postInitClosingEvents"_a = false,
                 "ouputDatabasePath"_a = "")
            .def("computeLikelihood", &Memory::computeLikelihood, "signature"_a, "ids"_a)
            .def("incrementMapId", &Memory::incrementMapId, "reducedIds"_a)
            .def("updateAge", &Memory::updateAge, "signatureId"_a)

            .def("forget", &Memory::forget, "ignoredIds"_a)
            .def("reactivateSignatures", &Memory::reactivateSignatures, "ids"_a, "maxLoaded"_a, "timeDbAccess"_a)

            .def("cleanup", &Memory::cleanup)
            .def("saveStatistics", &Memory::saveStatistics, "statistics"_a, "saveWMState"_a)
            .def("savePreviewImage", &Memory::savePreviewImage, "image"_a)
            .def("loadPreviewImage", &Memory::loadPreviewImage)
            .def("saveOptimizedPoses", &Memory::saveOptimizedPoses, "optimizedPoses"_a, "lastlocalizationPose"_a)
            .def("loadOptimizedPoses", &Memory::loadOptimizedPoses, "lastlocalizationPose"_a)
            .def("save2DMap", &Memory::save2DMap, "map"_a, "xMin"_a, "yMin"_a, "cellSize"_a)
            .def("load2DMap", &Memory::load2DMap, "xMin"_a, "yMin"_a, "cellSize"_a)
            .def("saveOptimizedMesh", &Memory::saveOptimizedMesh, "cloud"_a, "polygons"_a, "texCoords"_a, "textures"_a)
            .def("loadOptimizedMesh", &Memory::loadOptimizedMesh, "polygons"_a, "texCoords"_a, "textures"_a)
            .def("emptyTrash", &Memory::emptyTrash)
            .def("joinTrashThread", &Memory::joinTrashThread)
            .def("addLink", &Memory::addLink, "link"_a, "addInDatabase"_a)
            .def("updateLink", &Memory::updateLink, "link"_a, "updateInDatabase"_a)
            .def("removeAllVirtualLinks", &Memory::removeAllVirtualLinks)
            .def("removeVirtualLinks", &Memory::removeVirtualLinks, "signatureId"_a)
            .def("getNeighborsId", &Memory::getNeighborsId, "signatureId"_a, "maxGraphDepth"_a,
                 "maxCheckedInDatabase"_a = -1, "incrementMarginOnLoop"_a = false, "ignoreLoopIds"_a = false,
                 "ignoreIntermediateNodes"_a = false, "ignoreLocalSpaceLoopIds"_a = false,
                 "nodesSet"_a = std::set<int>(), "dbAccessTime"_a = 0)
            .def("getNeighborsIdRadius", &Memory::getNeighborsIdRadius, "signatureId"_a, "radius"_a, "optimizedPoses"_a,
                 "maxGraphDepth"_a)
            .def("deleteLocation", &Memory::deleteLocation, "locationId"_a, "deletedWords"_a)
            .def("saveLocationData", &Memory::saveLocationData, "locationId"_a)
            .def("removeLink", &Memory::removeLink, "idA"_a, "idB"_a)
            .def("removeRawData", &Memory::removeRawData, "id"_a, "image"_a = true, "scan"_a = true,
                 "userData"_a = true)

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
            .def("getNodesObservingLandmark", &Memory::getNodesObservingLandmark, "landmarkId"_a, "lookInDatabase"_a)
            .def("getSignatureIdByLabel", &Memory::getSignatureIdByLabel, "label"_a, "lookInDatabase"_a = true)
            .def("labelSignature", &Memory::labelSignature, "id"_a, "label"_a)
            .def("getAllLabels", &Memory::getAllLabels)
            .def("getLandmarksIndex", &Memory::getLandmarksIndex)
            .def("allNodesInWM", &Memory::allNodesInWM)

            .def("setUserData", &Memory::setUserData, "id"_a, "data"_a)
            .def("getDatabaseMemoryUsed", &Memory::getDatabaseMemoryUsed)
            .def("getDatabaseVersion", &Memory::getDatabaseVersion)
            .def("getDatabaseUrl", &Memory::getDatabaseUrl)
            .def("getDbSavingTime", &Memory::getDbSavingTime)
            .def("getMapId", &Memory::getMapId, "id"_a, "lookInDatabase"_a = false)
            .def("getOdomPose", &Memory::getOdomPose, "signatureId"_a, "lookInDatabase"_a = false)
            .def("getGroundTruthPose", &Memory::getGroundTruthPose, "signatureId"_a, "lookInDatabase"_a = false)
            .def("getGroundTruths", &Memory::getGroundTruths)
            .def("getGPS", &Memory::getGPS, "id"_a, "gps"_a, "offsetENU"_a, "lookInDatabase"_a, "maxGraphDepth"_a = 0)
            .def("getNodeInfo", &Memory::getNodeInfo, "signatureId"_a, "odomPose"_a, "mapId"_a, "weight"_a, "label"_a,
                 "stamp"_a, "groundTruth"_a, "velocity"_a, "gps"_a, "sensors"_a, "lookInDatabase"_a = false)
            .def("getImageCompressed", &Memory::getImageCompressed, "signatureId"_a)
            .def("getNodeData", &Memory::getNodeData, "locationId"_a, "images"_a, "scan"_a, "userData"_a,
                 "occupancyGrid"_a)
            // .def("getNodeWordsAndGlobalDescriptors", &Memory::getNodeWordsAndGlobalDescriptors)
            .def("getNodeCalibration", &Memory::getNodeCalibration, "nodeId"_a, "models"_a, "stereoModels"_a)
            .def("getAllSignatureIds", &Memory::getAllSignatureIds, "ignoreChildren"_a = true)
            .def("memoryChanged", &Memory::memoryChanged)
            .def("isIncremental", &Memory::isIncremental)
            .def("isLocalizationDataSaved", &Memory::isLocalizationDataSaved)
            .def("getSignature", &Memory::getSignature, "id"_a)
            .def("isInSTM", &Memory::isInSTM, "signatureId"_a)
            .def("isInWM", &Memory::isInWM, "signatureId"_a)
            .def("isInLTM", &Memory::isInLTM, "signatureId"_a)
            .def("isIDsGenerated", &Memory::isIDsGenerated)
            .def("getLastGlobalLoopClosureId", &Memory::getLastGlobalLoopClosureId)
            .def("getFeature2D", &Memory::getFeature2D)
            .def("isGraphReduced", &Memory::isGraphReduced)
            .def("getOdomMaxInf", &Memory::getOdomMaxInf)
            .def("isOdomGravityUsed", &Memory::isOdomGravityUsed)

            .def("dumpMemoryTree", &Memory::dumpMemoryTree, "fileNameTree"_a)
            .def("dumpMemory", &Memory::dumpMemory, "directory"_a)
            .def("dumpSignatures", &Memory::dumpSignatures, "fileNameSign"_a, "words3D"_a)
            .def("dumpDictionary", &Memory::dumpDictionary, "fileNameRef"_a, "fileNameDesc"_a)
            .def("getMemoryUsed", &Memory::getMemoryUsed)

            .def("generateGraph", &Memory::generateGraph, "fileName"_a, "ids"_a = std::set<int>())
            .def("cleanupLocalGrids", &Memory::cleanupLocalGrids, "poses"_a, "map"_a, "xMin"_a, "yMin"_a, "cellSize"_a,
                 "cropRadius"_a = 1, "filterScans"_a = false)
            .def("getVWDictionary", &Memory::getVWDictionary)
            // .def("getMetricConstraints", &Memory::getMetricConstraints)
            .def("computeTransform",
                 nb::overload_cast<Signature&, Signature&, Transform, RegistrationInfo*, bool>(
                         &Memory::computeTransform, nb::const_),
                 "fromS"_a, "toS"_a, "guess"_a, "info"_a, "useKnownCorrespondencesIfPossible"_a)
            .def("computeTransform",
                 nb::overload_cast<int, int, Transform, RegistrationInfo*, bool>(&Memory::computeTransform), "fromId"_a,
                 "toId"_a, "guess"_a, "info"_a, "useKnownCorrespondencesIfPossible"_a)
            .def("computeIcpTransform", &Memory::computeIcpTransform, "fromS"_a, "toS"_a, "guess"_a, "info"_a)
            .def("computeIcpTransformMulti", &Memory::computeIcpTransformMulti, "newId"_a, "oldId"_a, "poses"_a,
                 "info"_a);
}