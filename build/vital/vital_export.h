
#ifndef VITAL_EXPORT_H
#define VITAL_EXPORT_H

#ifdef VITAL_BUILD_AS_STATIC
#  define VITAL_EXPORT
#  define VITAL_NO_EXPORT
#else
#  ifndef VITAL_EXPORT
#    ifdef vital_EXPORTS
        /* We are building this library */
#      define VITAL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VITAL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VITAL_NO_EXPORT
#    define VITAL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VITAL_DEPRECATED
#  define VITAL_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef VITAL_DEPRECATED_EXPORT
#  define VITAL_DEPRECATED_EXPORT VITAL_EXPORT VITAL_DEPRECATED
#endif

#ifndef VITAL_DEPRECATED_NO_EXPORT
#  define VITAL_DEPRECATED_NO_EXPORT VITAL_NO_EXPORT VITAL_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef VITAL_NO_DEPRECATED
#    define VITAL_NO_DEPRECATED
#  endif
#endif

#endif
