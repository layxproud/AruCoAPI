#ifndef ARUCOAPI_GLOBAL_H
#define ARUCOAPI_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(ARUCOAPI_LIBRARY)
#  define ARUCOAPI_EXPORT Q_DECL_EXPORT
#else
#  define ARUCOAPI_EXPORT Q_DECL_IMPORT
#endif

#endif // ARUCOAPI_GLOBAL_H
