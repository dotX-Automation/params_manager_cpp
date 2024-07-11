"""
Parameters manager init_parameters.cpp generator.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Roberto Masocco <r.masocco@dotxautomation.com>

May 5, 2023
"""

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

import argparse
import os
import sys
import yaml

# C++ code templates and text substitutions
defaults = {
    'bool': '{{default_value}}',
    'double': '{{default_value}}, {{min_value}}, {{max_value}}, {{step}}',
    'integer': '{{default_value}}, {{min_value}}, {{max_value}}, {{step}}',
    'string': '"{{default_value}}"',
    'bool_array': '{{{default_value}}}',
    'double_array': '{{{default_value}}}, {{min_value}}, {{max_value}}, {{step}}',
    'integer_array': '{{{default_value}}}, {{min_value}}, {{max_value}}, {{step}}',
    'string_array': '{{{default_value}}}'
}

params_decl = """\
  // {{param_name}}
  {{manager_name}}->declare_{{type}}_parameter(
    "{{param_name}}",
    {{param_values}},
    "{{description}}",
    "{{constraints}}",
    {{read_only}},
    {{var_ptr}},
    {{validator}});

"""

validator_bind = """\
std::bind(
      &{{node_class_name}}::{{validator}},
      this,
      std::placeholders::_1)"""

cpp_code = """\
#include <{{header_include_path}}>
{{namespace1}}
/**
 * @brief Routine to initialize node parameters.
 */
void {{node_class_name}}::init_parameters()
{
{{params_declarations}}
}
{{namespace2}}"""

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('config_file',
                        type=str,
                        help='Configuration file path',
                        default='')
    parser.add_argument('out_file',
                        type=str,
                        help='Output file path',
                        default='')
    if len(sys.argv) != 3:
        parser.print_usage()
        exit(-1)
    args = parser.parse_args()
    yaml_file = args.config_file
    out_file = args.out_file

    # Open YAML file and read data
    try:
        with open(yaml_file) as f:
            yaml_data = yaml.load(f, Loader=yaml.SafeLoader)
    except:
        print('ERROR: Failed to parse YAML file')
        exit(-1)

    # Get general configuration
    header_include_path = yaml_data.get('header_include_path')
    if header_include_path == None:
        print('ERROR: Invalid header_include_path')
        exit(-1)
    manager_name = yaml_data.get('manager_name', 'pmanager_')
    namespace = yaml_data.get('namespace', '')
    node_class_name = yaml_data.get('node_class_name')
    if node_class_name == None:
        print('ERROR: Invalid node_class_name')
        exit(-1)

    # Get ordered params dictionary
    params_dict = dict(sorted(yaml_data['params'].items()))
    params_decls_cpp = ''
    for (param_name, values) in params_dict.items():
        try:
            # Generate parameter metadata
            # This highly depends on the parameter type and YAML parsing
            if values['type'] in ['integer', 'double']:
                param_values = defaults[values['type']].replace('{{default_value}}', str(values['default_value']))\
                    .replace('{{min_value}}', str(values['min_value']))\
                    .replace('{{max_value}}', str(values['max_value']))\
                    .replace('{{step}}', str(values['step']))
            elif values['type'] in ['integer_array', 'double_array']:
                param_values = defaults[values['type']].replace('{{min_value}}', str(values['min_value']))\
                    .replace('{{max_value}}', str(values['max_value']))\
                    .replace('{{step}}', str(values['step']))
                default_values = ''
                if len(list(values['default_value'])) > 0:
                    for i in range(0, len(list(values['default_value'])) - 1):
                        default_values += str(list(values['default_value'])[i]) + ', '
                    default_values += str(list(values['default_value'])[-1])
                param_values = param_values.replace('{{default_value}}', default_values)
            elif values['type'] in ['bool_array', 'string_array']:
                default_values = ''
                is_string = values['type'] == 'string_array'
                if len(list(values['default_value'])) > 0:
                    for i in range(0, len(list(values['default_value'])) - 1):
                        default_values += '"' if is_string else ''
                        default_values +=\
                            str(list(values['default_value'])[i]) if is_string\
                            else str(list(values['default_value'])[i]).lower()
                        default_values += '", ' if is_string else ', '
                    default_values += '"' if is_string else ''
                    default_values +=\
                        str(list(values['default_value'])[-1]) if is_string\
                        else str(list(values['default_value'])[-1]).lower()
                    default_values += '"' if is_string else ''
                param_values = defaults[values['type']].replace('{{default_value}}', default_values)
            else:  # bool, string
                is_string = values['type'] == 'string'
                param_values =\
                    defaults[values['type']].replace('{{default_value}}', str(values['default_value'])) if is_string\
                    else defaults[values['type']].replace('{{default_value}}', str(values['default_value']).lower())

            # Get parameter variable name, if present
            var_name = str(values.get('var_name', ''))
            if var_name != '':
                var_ptr_subst = '&' + var_name
            else:
                var_ptr_subst = 'nullptr'

            # Get parameter validator, if present
            validator_name = str(values.get('validator', ''))
            if validator_name != '':
                validator_subst = validator_bind.replace('{{node_class_name}}', node_class_name)\
                    .replace('{{validator}}', validator_name)
            else:
                validator_subst = 'nullptr'

            params_decls_cpp += params_decl.replace('{{description}}', values['description'])\
                .replace('{{type}}', values['type'])\
                .replace('{{param_name}}', param_name)\
                .replace('{{param_values}}', param_values)\
                .replace('{{description}}', values['description'])\
                .replace('{{constraints}}', values['constraints'])\
                .replace('{{manager_name}}', manager_name)\
                .replace('{{validator}}', validator_subst)\
                .replace('{{read_only}}', str(values['read_only']).lower())\
                .replace('{{var_ptr}}', var_ptr_subst)

        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(f'ERROR: {exc_type} at {fname}::{exc_tb.tb_lineno}')
            print(f'Failed to parse configuration of parameter: {param_name}')
            exit(-1)

    cpp_code = cpp_code.replace('{{params_declarations}}', params_decls_cpp[:-2])
    cpp_code = cpp_code.replace('{{node_class_name}}', node_class_name)
    cpp_code = cpp_code.replace('{{header_include_path}}', header_include_path)
    if namespace != '':
        namespace1 = '\nnamespace '+namespace+'\n{\n'
        namespace2 = '\n} // namespace '+namespace+'\n'
        cpp_code = cpp_code.replace('{{namespace1}}', namespace1)
        cpp_code = cpp_code.replace('{{namespace2}}', namespace2)
    else:
        cpp_code = cpp_code.replace('{{namespace1}}', namespace)
        cpp_code = cpp_code.replace('{{namespace2}}', namespace)

    # Write C++ source file
    with open(out_file, 'w') as f:
        f.write(cpp_code)
