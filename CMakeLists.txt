cmake_minimum_required(VERSION 3.31)
project(pyRTAB)

include("cmake/add_stub.cmake")

set(CMAKE_CXX_STANDARD 17)

# Try to import all Python components potentially needed by nanobind
find_package(Python 3.8
        REQUIRED COMPONENTS Interpreter Development.Module
        OPTIONAL_COMPONENTS Development.SABIModule)
execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)# Import nanobind through CMake's find_package mechanism
find_package(nanobind CONFIG REQUIRED)

add_subdirectory(third_party)

####################################################################################################
# We are now ready to compile the actual extension module
nanobind_add_module(
        # Name of the extension
        rtab_ext

        # Build libnanobind statically and merge it into the
        # extension (which itself remains a shared library)
        #
        # If your project builds multiple extensions, you can
        # replace this flag by NB_SHARED to conserve space by
        # reusing a shared libnanobind across libraries
        NB_STATIC

        # Source code goes here
        src/py_rtab_ext.cpp
        src/py_odometry.cpp
        src/py_transform.cpp
        src/py_camera_model.cpp
        src/py_sensor_data.cpp
        src/py_laser_scan.cpp
        src/py_stereo_camera_model.cpp
        src/py_statistics.cpp
        src/py_signature.cpp
        src/py_link.cpp
        src/py_memory.cpp
        src/py_feature2d.cpp
        src/py_vw_dictionary.cpp
        src/py_visual_word.cpp
)

add_stub(
        rtab_stub
        MODULE rtab_ext
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/py_rtab
        PYTHON_PATH $<TARGET_FILE_DIR:rtab_ext>
        DEPENDS rtab_ext
)

target_link_libraries(rtab_ext PRIVATE
        rtabmap_core
)

######################
# install libraries
######################
install(TARGETS rtab_ext LIBRARY DESTINATION py_rtab)

file(GLOB PY_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/py_rtab/*.py" "${CMAKE_CURRENT_SOURCE_DIR}/src/py_rtab/*.pyi")
install(FILES ${PY_FILES} DESTINATION py_rtab)