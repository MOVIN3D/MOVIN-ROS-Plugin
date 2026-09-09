cmake_minimum_required(VERSION 3.8)
project(movin_sdk_python_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(MOVIN_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/MOVIN-SDK-Python")
set(MOVIN_SDK_PACKAGE_DIR "${MOVIN_SDK_DIR}/movin_sdk_python")

if(NOT EXISTS "${MOVIN_SDK_PACKAGE_DIR}/__init__.py")
  message(FATAL_ERROR
    "MOVIN-SDK-Python submodule is not initialized: ${MOVIN_SDK_PACKAGE_DIR} is missing.\n"
    "Run: git submodule update --init --recursive")
endif()

# Read the SDK's own version so the installed egg-info matches movin_sdk_python.__version__.
file(READ "${MOVIN_SDK_PACKAGE_DIR}/__init__.py" _movin_sdk_init)
string(REGEX MATCH "__version__ = \"([0-9]+\\.[0-9]+\\.[0-9]+)\"" _movin_sdk_version_match "${_movin_sdk_init}")
if(CMAKE_MATCH_1)
  set(MOVIN_SDK_VERSION "${CMAKE_MATCH_1}")
else()
  set(MOVIN_SDK_VERSION "0.1.0")
endif()
message(STATUS "Vendoring movin_sdk_python ${MOVIN_SDK_VERSION} from ${MOVIN_SDK_PACKAGE_DIR}")

# Installs the whole package directory (including retargeter/ik_configs/*.json and
# retargeter/assets/**/*.xml,*.STL) into this package's Python site-packages and
# puts it on PYTHONPATH through the ament environment hooks. The submodule's own
# setup.py / pyproject.toml are never executed.
ament_python_install_package(movin_sdk_python
  PACKAGE_DIR "${MOVIN_SDK_PACKAGE_DIR}"
  VERSION "${MOVIN_SDK_VERSION}"
)

ament_package()
