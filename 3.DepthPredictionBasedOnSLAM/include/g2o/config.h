#ifndef G2O_CONFIG_H
#define G2O_CONFIG_H


#define G2O_HAVE_OPENGL 1
#define G2O_OPENGL_FOUND 1
#define G2O_OPENMP 1
#define G2O_SHARED_LIBS 1
#define G2O_LGPL_SHARED_LIBS 1

// available sparse matrix libraries
/* #undef G2O_HAVE_CHOLMOD */
#define G2O_HAVE_CSPARSE 1

/* #undef G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS */

#ifdef G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS
#define G2O_DELETE_IMPLICITLY_OWNED_OBJECTS 0
#else
#define G2O_DELETE_IMPLICITLY_OWNED_OBJECTS 1
#endif

/* #undef G2O_SINGLE_PRECISION_MATH */
#ifdef G2O_SINGLE_PRECISION_MATH
    #define G2O_NUMBER_FORMAT_STR "%g"

    #ifdef __cplusplus
        using number_t = float;
    #else
        typedef float number_t;
    #endif
#else
    #define G2O_NUMBER_FORMAT_STR "%lg"

    #ifdef __cplusplus
        using number_t = double;
    #else
        typedef double number_t;
    #endif
#endif

#define G2O_CXX_COMPILER "MSVC C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/VC/Tools/MSVC/14.16.27023/bin/Hostx86/x64/cl.exe"

#ifdef __cplusplus
#include <g2o/core/eigen_types.h>
#endif

#endif
