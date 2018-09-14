
// -*- C++ -*-
// $Id$
// Definition for Win32 Export directives.
// This file is generated automatically by generate_export_file.pl knMotor
// ------------------------------
#ifndef KNMOTOR_EXPORT_H
#define KNMOTOR_EXPORT_H

#include "ace/config-all.h"

#if defined (ACE_AS_STATIC_LIBS) && !defined (KNMOTOR_HAS_DLL)
#  define KNMOTOR_HAS_DLL 0
#endif /* ACE_AS_STATIC_LIBS && KNMOTOR_HAS_DLL */

#if !defined (KNMOTOR_HAS_DLL)
#  define KNMOTOR_HAS_DLL 1
#endif /* ! KNMOTOR_HAS_DLL */

#if defined (KNMOTOR_HAS_DLL) && (KNMOTOR_HAS_DLL == 1)
#  if defined (KNMOTOR_BUILD_DLL)
#    define knMotor_Export ACE_Proper_Export_Flag
#    define KNMOTOR_SINGLETON_DECLARATION(T) ACE_EXPORT_SINGLETON_DECLARATION (T)
#    define KNMOTOR_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_EXPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  else /* KNMOTOR_BUILD_DLL */
#    define knMotor_Export ACE_Proper_Import_Flag
#    define KNMOTOR_SINGLETON_DECLARATION(T) ACE_IMPORT_SINGLETON_DECLARATION (T)
#    define KNMOTOR_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_IMPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  endif /* KNMOTOR_BUILD_DLL */
#else /* KNMOTOR_HAS_DLL == 1 */
#  define knMotor_Export
#  define KNMOTOR_SINGLETON_DECLARATION(T)
#  define KNMOTOR_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#endif /* KNMOTOR_HAS_DLL == 1 */

// Set KNMOTOR_NTRACE = 0 to turn on library specific tracing even if
// tracing is turned off for ACE.
#if !defined (KNMOTOR_NTRACE)
#  if (ACE_NTRACE == 1)
#    define KNMOTOR_NTRACE 1
#  else /* (ACE_NTRACE == 1) */
#    define KNMOTOR_NTRACE 0
#  endif /* (ACE_NTRACE == 1) */
#endif /* !KNMOTOR_NTRACE */

#if (KNMOTOR_NTRACE == 1)
#  define KNMOTOR_TRACE(X)
#else /* (KNMOTOR_NTRACE == 1) */
#  if !defined (ACE_HAS_TRACE)
#    define ACE_HAS_TRACE
#  endif /* ACE_HAS_TRACE */
#  define KNMOTOR_TRACE(X) ACE_TRACE_IMPL(X)
#  include "ace/Trace.h"
#endif /* (KNMOTOR_NTRACE == 1) */

#endif /* KNMOTOR_EXPORT_H */

// End of auto generated file.
