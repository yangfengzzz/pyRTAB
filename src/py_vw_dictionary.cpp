//  Copyright (c) 2025 Feng Yang
//
//  I am making my contributions/submissions to this project solely in my
//  personal capacity and am not conveying any rights to any intellectual
//  property of any third parties.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/list.h>
#include <nanobind/stl/vector.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/VisualWord.h>
#include "cv_typecaster.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace rtabmap;

void bindVWDictionary(nb::module_ &m) {
    nb::class_<VWDictionary>(m, "VWDictionary")
            .def(nb::init<>())
            .def("parseParameters", &VWDictionary::parseParameters, "parameters"_a)
            .def("update", &VWDictionary::update)
            .def("addNewWords", &VWDictionary::addNewWords, "descriptors"_a, "signatureId"_a)
            .def("addWord", &VWDictionary::addWord, "vw"_a)
            .def("findNN", nb::overload_cast<const cv::Mat &>(&VWDictionary::findNN, nb::const_), "vws"_a)
            .def("addWordRef", &VWDictionary::addWordRef, "wordId"_a, "signatureId"_a)
            .def("removeAllWordRef", &VWDictionary::removeAllWordRef, "wordId"_a, "signatureId"_a)
            .def("getWord", &VWDictionary::getWord, "id"_a)
            .def("getUnusedWord", &VWDictionary::getUnusedWord, "id"_a)
            .def("setLastWordId", &VWDictionary::setLastWordId, "id"_a)
            .def("getVisualWords", &VWDictionary::getVisualWords)
            .def("getNndrRatio", &VWDictionary::getNndrRatio)
            .def("getNotIndexedWordsCount", &VWDictionary::getNotIndexedWordsCount)
            .def("getLastIndexedWordId", &VWDictionary::getLastIndexedWordId)
            .def("getTotalActiveReferences", &VWDictionary::getTotalActiveReferences)
            .def("getIndexedWordsCount", &VWDictionary::getIndexedWordsCount)
            .def("getIndexMemoryUsed", &VWDictionary::getIndexMemoryUsed)
            .def("getMemoryUsed", &VWDictionary::getMemoryUsed)
            .def("setNNStrategy", &VWDictionary::setNNStrategy, "strategy"_a)
            .def("isIncremental", &VWDictionary::isIncremental)
            .def("isIncrementalFlann", &VWDictionary::isIncrementalFlann)
            .def("setIncrementalDictionary", &VWDictionary::setIncrementalDictionary)
            .def("setFixedDictionary", &VWDictionary::setFixedDictionary, "dictionaryPath"_a)
            .def("isModified", &VWDictionary::isModified)
            .def("serializeIndex", &VWDictionary::serializeIndex)
            .def("deserializeIndex",
                 nb::overload_cast<const std::vector<unsigned char> &>(&VWDictionary::deserializeIndex), "data"_a)
            .def("exportDictionary", &VWDictionary::exportDictionary, "fileNameReferences"_a, "fileNameDescriptors"_a)
            .def("clear", &VWDictionary::clear, "printWarningsIfNotEmpty"_a = true)
            .def("getUnusedWords", &VWDictionary::getUnusedWords)
            .def("getUnusedWordIds", &VWDictionary::getUnusedWordIds)
            .def("getUnusedWordsSize", &VWDictionary::getUnusedWordsSize)
            .def("removeWords", &VWDictionary::removeWords, "words"_a)
            .def("deleteUnusedWords", &VWDictionary::deleteUnusedWords)
            .def_static("convertBinTo32F", &VWDictionary::convertBinTo32F, "descriptorsIn"_a, "byteToFloat"_a = true)
            .def_static("convert32FToBin", &VWDictionary::convert32FToBin, "descriptorsIn"_a, "byteToFloat"_a = true);

    nb::enum_<VWDictionary::NNStrategy>(m, "NNStrategy")
            .value("kNNFlannNaive", VWDictionary::NNStrategy::kNNFlannNaive)
            .value("kNNFlannKdTree", VWDictionary::NNStrategy::kNNFlannKdTree)
            .value("kNNFlannLSH", VWDictionary::NNStrategy::kNNFlannLSH)
            .value("kNNBruteForce", VWDictionary::NNStrategy::kNNBruteForce)
            .value("kNNBruteForceGPU", VWDictionary::NNStrategy::kNNBruteForceGPU)
            .value("kNNUndef", VWDictionary::NNStrategy::kNNUndef);
}