#ifndef ARDUPILOT_MSGS__VISIBILITY_CONTROL_H_
#define ARDUPILOT_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ARDUPILOT_MSGS_EXPORT __attribute__ ((dllexport))
    #define ARDUPILOT_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define ARDUPILOT_MSGS_EXPORT __declspec(dllexport)
    #define ARDUPILOT_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ARDUPILOT_MSGS_BUILDING_LIBRARY
    #define ARDUPILOT_MSGS_PUBLIC ARDUPILOT_MSGS_EXPORT
  #else
    #define ARDUPILOT_MSGS_PUBLIC ARDUPILOT_MSGS_IMPORT
  #endif
  #define ARDUPILOT_MSGS_PUBLIC_TYPE ARDUPILOT_MSGS_PUBLIC
  #define ARDUPILOT_MSGS_LOCAL
#else
  #define ARDUPILOT_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define ARDUPILOT_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define ARDUPILOT_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define ARDUPILOT_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ARDUPILOT_MSGS_PUBLIC
    #define ARDUPILOT_MSGS_LOCAL
  #endif
  #define ARDUPILOT_MSGS_PUBLIC_TYPE
#endif
#endif  // ARDUPILOT_MSGS__VISIBILITY_CONTROL_H_
// Generated 08-May-2026 14:44:02
 