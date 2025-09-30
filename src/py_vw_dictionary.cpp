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
            .def("parseParameters", &VWDictionary::parseParameters)
            .def("update", &VWDictionary::update)
            .def("addNewWords", &VWDictionary::addNewWords)
            .def("addWord", &VWDictionary::addWord)
            .def("findNN", nb::overload_cast<const cv::Mat &>(&VWDictionary::findNN, nb::const_))
            .def("addWordRef", &VWDictionary::addWordRef)
            .def("removeAllWordRef", &VWDictionary::removeAllWordRef)
            .def("getWord", &VWDictionary::getWord)
            .def("getUnusedWord", &VWDictionary::getUnusedWord)
            .def("setLastWordId", &VWDictionary::setLastWordId)
            .def("getVisualWords", &VWDictionary::getVisualWords)
            .def("getNndrRatio", &VWDictionary::getNndrRatio)
            .def("getNotIndexedWordsCount", &VWDictionary::getNotIndexedWordsCount)
            .def("getLastIndexedWordId", &VWDictionary::getLastIndexedWordId)
            .def("getTotalActiveReferences", &VWDictionary::getTotalActiveReferences)
            .def("getIndexedWordsCount", &VWDictionary::getIndexedWordsCount)
            .def("getIndexMemoryUsed", &VWDictionary::getIndexMemoryUsed)
            .def("getMemoryUsed", &VWDictionary::getMemoryUsed)
            .def("setNNStrategy", &VWDictionary::setNNStrategy)
            .def("isIncremental", &VWDictionary::isIncremental)
            .def("isIncrementalFlann", &VWDictionary::isIncrementalFlann)
            .def("setIncrementalDictionary", &VWDictionary::setIncrementalDictionary)
            .def("setFixedDictionary", &VWDictionary::setFixedDictionary)
            .def("isModified", &VWDictionary::isModified)
            .def("serializeIndex", &VWDictionary::serializeIndex)
            .def("deserializeIndex",
                 nb::overload_cast<const std::vector<unsigned char> &>(&VWDictionary::deserializeIndex))
            .def("exportDictionary", &VWDictionary::exportDictionary)
            .def("clear", &VWDictionary::clear)
            .def("getUnusedWords", &VWDictionary::getUnusedWords)
            .def("getUnusedWordIds", &VWDictionary::getUnusedWordIds)
            .def("getUnusedWordsSize", &VWDictionary::getUnusedWordsSize)
            .def("removeWords", &VWDictionary::removeWords)
            .def("deleteUnusedWords", &VWDictionary::deleteUnusedWords)
            .def("convertBinTo32F", &VWDictionary::convertBinTo32F)
            .def("convert32FToBin", &VWDictionary::convert32FToBin);

    nb::enum_<VWDictionary::NNStrategy>(m, "NNStrategy")
            .value("kNNFlannNaive", VWDictionary::NNStrategy::kNNFlannNaive)
            .value("kNNFlannKdTree", VWDictionary::NNStrategy::kNNFlannKdTree)
            .value("kNNFlannLSH", VWDictionary::NNStrategy::kNNFlannLSH)
            .value("kNNBruteForce", VWDictionary::NNStrategy::kNNBruteForce)
            .value("kNNBruteForceGPU", VWDictionary::NNStrategy::kNNBruteForceGPU)
            .value("kNNUndef", VWDictionary::NNStrategy::kNNUndef);
}