#ifndef NFR_DEPTH_FINDER__VISIBILITY_CONTROL_H_
#define NFR_DEPTH_FINDER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NFR_DEPTH_FINDER_EXPORT __attribute__ ((dllexport))
    #define NFR_DEPTH_FINDER_IMPORT __attribute__ ((dllimport))
  #else
    #define NFR_DEPTH_FINDER_EXPORT __declspec(dllexport)
    #define NFR_DEPTH_FINDER_IMPORT __declspec(dllimport)
  #endif
  #ifdef NFR_DEPTH_FINDER_BUILDING_LIBRARY
    #define NFR_DEPTH_FINDER_PUBLIC NFR_DEPTH_FINDER_EXPORT
  #else
    #define NFR_DEPTH_FINDER_PUBLIC NFR_DEPTH_FINDER_IMPORT
  #endif
  #define NFR_DEPTH_FINDER_PUBLIC_TYPE NFR_DEPTH_FINDER_PUBLIC
  #define NFR_DEPTH_FINDER_LOCAL
#else
  #define NFR_DEPTH_FINDER_EXPORT __attribute__ ((visibility("default")))
  #define NFR_DEPTH_FINDER_IMPORT
  #if __GNUC__ >= 4
    #define NFR_DEPTH_FINDER_PUBLIC __attribute__ ((visibility("default")))
    #define NFR_DEPTH_FINDER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NFR_DEPTH_FINDER_PUBLIC
    #define NFR_DEPTH_FINDER_LOCAL
  #endif
  #define NFR_DEPTH_FINDER_PUBLIC_TYPE
#endif

#endif  // NFR_DEPTH_FINDER__VISIBILITY_CONTROL_H_
