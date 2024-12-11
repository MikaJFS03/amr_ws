#ifndef EDABOT_COMMUNICATION__VISIBILITY_CONTROL_H_
#define EDABOT_COMMUNICATION__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EDABOT_COMMUNICATION_EXPORT __attribute__ ((dllexport))
    #define EDABOT_COMMUNICATION_IMPORT __attribute__ ((dllimport))
  #else
    #define EDABOT_COMMUNICATION_EXPORT __declspec(dllexport)
    #define EDABOT_COMMUNICATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef EDABOT_COMMUNICATION_BUILDING_DLL
    #define EDABOT_COMMUNICATION_PUBLIC EDABOT_COMMUNICATION_EXPORT
  #else
    #define EDABOT_COMMUNICATION_PUBLIC EDABOT_COMMUNICATION_IMPORT
  #endif
  #define EDABOT_COMMUNICATION_PUBLIC_TYPE EDABOT_COMMUNICATION_PUBLIC
  #define EDABOT_COMMUNICATION_LOCAL
#else
  #define EDABOT_COMMUNICATION_EXPORT __attribute__ ((visibility("default")))
  #define EDABOT_COMMUNICATION_IMPORT
  #if __GNUC__ >= 4
    #define EDABOT_COMMUNICATION_PUBLIC __attribute__ ((visibility("default")))
    #define EDABOT_COMMUNICATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EDABOT_COMMUNICATION_PUBLIC
    #define EDABOT_COMMUNICATION_LOCAL
  #endif
  #define EDABOT_COMMUNICATION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // EDABOT_COMMUNICATION__VISIBILITY_CONTROL_H_