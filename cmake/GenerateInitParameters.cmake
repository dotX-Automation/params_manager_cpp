# CMake function code to call the init_parameters.cpp file generation script at build time.
#
# Roberto Masocco <r.masocco@dotxautomation.com>
#
# May 8, 2023

# Copyright 2024 dotX Automation s.r.l.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

function(generate_init_parameters)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)

  ament_index_has_resource(
    PARAMS_MANAGER_CPP_PREFIX
    packages
    params_manager_cpp)
  if(NOT PARAMS_MANAGER_CPP_PREFIX)
    message(FATAL_ERROR "Could not find params_manager_cpp package")
  endif()

  set(options "")
  set(oneValueArgs YAML_FILE OUT_FILE)
  set(multiValueArgs "")
  cmake_parse_arguments(
    GENERATE_INIT_PARAMETERS
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  if(NOT GENERATE_INIT_PARAMETERS_YAML_FILE)
    message(FATAL_ERROR "YAML_FILE argument not provided")
  endif()
  if(NOT GENERATE_INIT_PARAMETERS_OUT_FILE)
    message(FATAL_ERROR "OUT_FILE argument not provided")
  endif()

  add_custom_command(
    OUTPUT "${GENERATE_INIT_PARAMETERS_OUT_FILE}"
    COMMAND ${Python3_EXECUTABLE}
      "${PARAMS_MANAGER_CPP_PREFIX}/share/params_manager_cpp/scripts/generate_init_parameters.py"
      "${GENERATE_INIT_PARAMETERS_YAML_FILE}"
      "${CMAKE_CURRENT_BINARY_DIR}/${GENERATE_INIT_PARAMETERS_OUT_FILE}"
    MAIN_DEPENDENCY "${GENERATE_INIT_PARAMETERS_YAML_FILE}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Generating parameters declaration source file ${GENERATE_INIT_PARAMETERS_OUT_FILE} from ${GENERATE_INIT_PARAMETERS_YAML_FILE}"
    VERBATIM
    USES_TERMINAL)
endfunction()
