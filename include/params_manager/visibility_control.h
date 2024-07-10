#ifndef PARAMS_MANAGER__VISIBILITY_CONTROL_H_
#define PARAMS_MANAGER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PARAMS_MANAGER_EXPORT __attribute__ ((dllexport))
    #define PARAMS_MANAGER_IMPORT __attribute__ ((dllimport))
  #else
    #define PARAMS_MANAGER_EXPORT __declspec(dllexport)
    #define PARAMS_MANAGER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PARAMS_MANAGER_BUILDING_LIBRARY
    #define PARAMS_MANAGER_PUBLIC PARAMS_MANAGER_EXPORT
  #else
    #define PARAMS_MANAGER_PUBLIC PARAMS_MANAGER_IMPORT
  #endif
  #define PARAMS_MANAGER_PUBLIC_TYPE PARAMS_MANAGER_PUBLIC
  #define PARAMS_MANAGER_LOCAL
#else
  #define PARAMS_MANAGER_EXPORT __attribute__ ((visibility("default")))
  #define PARAMS_MANAGER_IMPORT
  #if __GNUC__ >= 4
    #define PARAMS_MANAGER_PUBLIC __attribute__ ((visibility("default")))
    #define PARAMS_MANAGER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PARAMS_MANAGER_PUBLIC
    #define PARAMS_MANAGER_LOCAL
  #endif
  #define PARAMS_MANAGER_PUBLIC_TYPE
#endif

#endif  // PARAMS_MANAGER__VISIBILITY_CONTROL_H_
