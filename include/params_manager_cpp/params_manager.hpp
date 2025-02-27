/**
 * ROS 2 parameter API wrapper.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 *
 * April 28, 2023
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PARAMS_MANAGER__PARAMSMANAGER_HPP_
#define PARAMS_MANAGER__PARAMSMANAGER_HPP_

#include "visibility_control.h"

#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

/* Function used to define a custom parameter validation procedure. */
using Validator = std::function<bool (const rclcpp::Parameter &)>;

namespace params_manager
{

/* Keeps this library coherent with the ParameterType ROS message. */
enum class PARAMS_MANAGER_LOCAL PType : uint8_t
{
  PARAMETER_NOT_SET = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET,
  PARAMETER_BOOL = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
  PARAMETER_INTEGER = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
  PARAMETER_DOUBLE = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
  PARAMETER_STRING = rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
  PARAMETER_BYTE_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY,
  PARAMETER_BOOL_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY,
  PARAMETER_INTEGER_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY,
  PARAMETER_DOUBLE_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY,
  PARAMETER_STRING_ARRAY = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY
};

/**
 * Stores parameter metadata required in this module.
 */
struct PARAMS_MANAGER_LOCAL ParamData
{
  std::string name_ = "";
  PType type_ = PType::PARAMETER_NOT_SET;
  Validator validator_ = nullptr;
  void * var_ptr_ = nullptr;

  /**
   * Constructor.
   */
  ParamData() {}

  /**
   * Constructor.
   *
   * @param name Parameter name.
   * @param type Parameter type.
   * @param var_ptr Pointer to the variable to be updated.
   * @param validator External parameter validation routine.
   */
  ParamData(const std::string & name, PType type, void * var_ptr, const Validator & validator)
  {
    name_ = name;
    type_ = type;
    validator_ = validator;
    var_ptr_ = var_ptr;
  }
};

/**
 * Defines the comparison operator for the parameter set.
 */
struct PARAMS_MANAGER_LOCAL PMComparator
{
  bool PARAMS_MANAGER_LOCAL operator()(
    const std::pair<std::string, ParamData> & lhs,
    const std::pair<std::string, ParamData> & rhs) const
  {
    return lhs.first < rhs.first;
  }
};

/**
 * Simplifies interactions with the ROS 2 parameter API.
 */
class PARAMS_MANAGER_PUBLIC Manager
{
public:
  /**
   * Constructor.
   *
   * @param node Pointer to the node to which this manager is attached.
   * @param verbose Activates event logging.
   *
   * @throws InvalidArgument if the node pointer is null.
   * @throws RuntimeError if callback setting fails.
   */
  Manager(rclcpp::Node * node, bool verbose = false);

  /**
   * Destructor.
   */
  virtual ~Manager();

  /**
   * @brief Routine to declare a boolean node parameter.
   *
   * @param name Parameter name.
   * @param default_val Default value.
   * @param desc Parameter description.
   * @param constraints Additional value constraints.
   * @param read_only Read-only internal flag.
   * @param var Pointer to the variable to be updated.
   * @param validator Parameter validation routine.
   *
   * @throws InvalidArgument if the parameter is already declared.
   */
  void declare_bool_parameter(
    std::string && name,
    bool default_val,
    std::string && desc, std::string && constraints, bool read_only,
    bool * var = nullptr,
    Validator && validator = nullptr);

  /**
   * @brief Routine to declare a boolean array node parameter.
   *
   * @param name Parameter name.
   * @param default_val Default value.
   * @param desc Parameter description.
   * @param constraints Additional value constraints.
   * @param read_only Read-only internal flag.
   * @param var Pointer to the variable to be updated.
   * @param validator Parameter validation routine.
   *
   * @throws InvalidArgument if the parameter is already declared.
   */
  void declare_bool_array_parameter(
    std::string && name,
    std::vector<bool> && default_val,
    std::string && desc, std::string && constraints, bool read_only,
    std::vector<bool> * var = nullptr,
    Validator && validator = nullptr);

  /**
   * @brief Routine to declare an integer node parameter.
   *
   * @param name Parameter name.
   * @param default_val Default value.
   * @param from Integer range initial value.
   * @param to Integer range final value.
   * @param step Integer range step.
   * @param desc Parameter description.
   * @param constraints Additional value constraints.
   * @param read_only Read-only internal flag.
   * @param var Pointer to the variable to be updated.
   * @param validator Paramter validation routine.
   *
   * @throws InvalidArgument if the parameter is already declared.
   */
  void declare_integer_parameter(
    std::string && name,
    int64_t default_val, int64_t from, int64_t to, int64_t step,
    std::string && desc, std::string && constraints, bool read_only,
    int64_t * var = nullptr,
    Validator && validator = nullptr);

  /**
   * @brief Routine to declare an integer array node parameter.
   *
   * @param name Parameter name.
   * @param default_val Default value.
   * @param from Integer range initial value.
   * @param to Integer range final value.
   * @param step Integer range step.
   * @param desc Parameter description.
   * @param constraints Additional value constraints.
   * @param read_only Read-only internal flag.
   * @param var Pointer to the variable to be updated.
   * @param validator Paramter validation routine.
   *
   * @throws InvalidArgument if the parameter is already declared.
   */
  void declare_integer_array_parameter(
    std::string && name,
    std::vector<int64_t> && default_val, int64_t from, int64_t to, int64_t step,
    std::string && desc, std::string && constraints, bool read_only,
    std::vector<int64_t> * var = nullptr,
    Validator && validator = nullptr);

  /**
   * @brief Routine to declare a 64-bit floating point node parameter.
   *
   * @param name Parameter name.
   * @param default_val Default value.
   * @param from Floating point range initial value.
   * @param to Floating point range final value.
   * @param step Floating point range step.
   * @param desc Parameter description.
   * @param constraints Additional value constraints.
   * @param read_only Read-only internal flag.
   * @param var Pointer to the variable to be updated.
   * @param validator Parameter validation routine.
   *
   * @throws InvalidArgument if the parameter is already declared.
   */
  void declare_double_parameter(
    std::string && name,
    double default_val, double from, double to, double step,
    std::string && desc, std::string && constraints, bool read_only,
    double * var = nullptr,
    Validator && validator = nullptr);

  /**
   * @brief Routine to declare a 64-bit floating point array node parameter.
   *
   * @param name Parameter name.
   * @param default_val Default value.
   * @param from Floating point range initial value.
   * @param to Floating point range final value.
   * @param step Floating point range step.
   * @param desc Parameter description.
   * @param constraints Additional value constraints.
   * @param read_only Read-only internal flag.
   * @param var Pointer to the variable to be updated.
   * @param validator Parameter validation routine.
   *
   * @throws InvalidArgument if the parameter is already declared.
   */
  void declare_double_array_parameter(
    std::string && name,
    std::vector<double> && default_val, double from, double to, double step,
    std::string && desc, std::string && constraints, bool read_only,
    std::vector<double> * var = nullptr,
    Validator && validator = nullptr);

  /**
   * @brief Routine to declare a string node parameter.
   *
   * @param name Parameter name.
   * @param default_val Default value.
   * @param desc Parameter description.
   * @param constraints Additional value constraints.
   * @param read_only Read-only internal flag.
   * @param var Pointer to the variable to be updated.
   * @param validator Parameter validation routine.
   *
   * @throws InvalidArgument if the parameter is already declared.
   */
  void declare_string_parameter(
    std::string && name,
    std::string && default_val,
    std::string && desc, std::string && constraints, bool read_only,
    std::string * var = nullptr,
    Validator && validator = nullptr);

  /**
   * @brief Routine to declare a string array node parameter.
   *
   * @param name Parameter name.
   * @param default_val Default value.
   * @param desc Parameter description.
   * @param constraints Additional value constraints.
   * @param read_only Read-only internal flag.
   * @param var Pointer to the variable to be updated.
   * @param validator Parameter validation routine.
   *
   * @throws InvalidArgument if the parameter is already declared.
   */
  void declare_string_array_parameter(
    std::string && name,
    std::vector<std::string> && default_val,
    std::string && desc, std::string && constraints, bool read_only,
    std::vector<std::string> * var = nullptr,
    Validator && validator = nullptr);

  /**
   * @brief Routine to declare a byte array node parameter.
   *
   * @param name Parameter name.
   * @param default_val Default value.
   * @param desc Parameter description.
   * @param constraints Additional value constraints.
   * @param read_only Read-only internal flag.
   * @param var Pointer to the variable to be updated.
   * @param validator Parameter validation routine.
   *
   * @throws InvalidArgument if the parameter is already declared.
   */
  void declare_byte_array_parameter(
    std::string && name,
    std::vector<uint8_t> && default_val,
    std::string && desc, std::string && constraints, bool read_only,
    std::vector<uint8_t> * var = nullptr,
    Validator && validator = nullptr);

  typedef std::shared_ptr<Manager> SharedPtr;

private:
  /* Operational flags. */
  bool verbose_ = false;

  /* Reference to node using this object. */
  rclcpp::Node * node_ = nullptr;

  /* Set of parameters being managed by this object. */
  std::set<std::pair<std::string, ParamData>, PMComparator> params_set_;

  /* Mutex used to protect the parameters set. */
  std::mutex params_set_lock_;

  /* Parameters callback handle. */
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr params_clbk_handle_ = nullptr;

  /**
   * @brief Callback routine for parameter setting.
   *
   * @param params Parameters to be set.
   *
   * @return SetParametersResult object to be forwarded to the ROS 2 parameters subsystem.
   */
  rcl_interfaces::msg::SetParametersResult PARAMS_MANAGER_LOCAL on_set_parameters_callback_(
    const std::vector<rclcpp::Parameter> & params);

  /**
   * @brief Adds a parameter to the parameter set.
   *
   * @param name Parameter name.
   * @param type Parameter type.
   * @param var_ptr Pointer to the variable to be updated.
   * @param validator External parameter validation routine.
   */
  void PARAMS_MANAGER_LOCAL add_to_set_(
    const std::string & name,
    PType type,
    void * var_ptr,
    const Validator & validator);

  /**
   * @brief Returns the parameter metadata for the given paramenter name, if present.
   *
   * @param name Parameter name.
   *
   * @return Pointer to the parameter metadata, or null if not present.
   */
  std::shared_ptr<ParamData> PARAMS_MANAGER_LOCAL get_param_data_(const std::string & name);

  /**
   * @brief Logs a parameter update event.
   *
   * @param p Parameter to be logged.
   */
  void PARAMS_MANAGER_LOCAL log_update_(const rclcpp::Parameter & param);
};

} // namespace params_manager

#endif // PARAMS_MANAGER__PARAMSMANAGER_HPP_
