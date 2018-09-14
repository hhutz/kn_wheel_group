
// -*- C++ -*-
// $Id$
// Definition for Win32 Export directives.
// This file is generated automatically by generate_export_file.pl knSimMotor
// ------------------------------
#ifndef KNSIMMOTOR_EXPORT_H
#define KNSIMMOTOR_EXPORT_H

#include "ace/config-all.h"

#if defined (ACE_AS_STATIC_LIBS) && !defined (KNSIMMOTOR_HAS_DLL)
#  define KNSIMMOTOR_HAS_DLL 0
#endif /* ACE_AS_STATIC_LIBS && KNSIMMOTOR_HAS_DLL */

#if !defined (KNSIMMOTOR_HAS_DLL)
#  define KNSIMMOTOR_HAS_DLL 1
#endif /* ! KNSIMMOTOR_HAS_DLL */

#if defined (KNSIMMOTOR_HAS_DLL) && (KNSIMMOTOR_HAS_DLL == 1)
#  if defined (KNSIMMOTOR_BUILD_DLL)
#    define knSimMotor_Export ACE_Proper_Export_Flag
#    define KNSIMMOTOR_SINGLETON_DECLARATION(T) ACE_EXPORT_SINGLETON_DECLARATION (T)
#    define KNSIMMOTOR_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_EXPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  else /* KNSIMMOTOR_BUILD_DLL */
#    define knSimMotor_Export ACE_Proper_Import_Flag
#    define KNSIMMOTOR_SINGLETON_DECLARATION(T) ACE_IMPORT_SINGLETON_DECLARATION (T)
#    define KNSIMMOTOR_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_IMPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  endif /* KNSIMMOTOR_BUILD_DLL */
#else /* KNSIMMOTOR_HAS_DLL == 1 */
#  define knSimMotor_Export
#  define KNSIMMOTOR_SINGLETON_DECLARATION(T)
#  define KNSIMMOTOR_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#endif /* KNSIMMOTOR_HAS_DLL == 1 */

// Set KNSIMMOTOR_NTRACE = 0 to turn on library specific tracing even if
// tracing is turned off for ACE.
#if !defined (KNSIMMOTOR_NTRACE)
#  if (ACE_NTRACE == 1)
#    define KNSIMMOTOR_NTRACE 1
#  else /* (ACE_NTRACE == 1) */
#    define KNSIMMOTOR_NTRACE 0
#  endif /* (ACE_NTRACE == 1) */
#endif /* !KNSIMMOTOR_NTRACE */

#if (KNSIMMOTOR_NTRACE == 1)
#  define KNSIMMOTOR_TRACE(X)
#else /* (KNSIMMOTOR_NTRACE == 1) */
#  if !defined (ACE_HAS_TRACE)
#    define ACE_HAS_TRACE
#  endif /* ACE_HAS_TRACE */
#  define KNSIMMOTOR_TRACE(X) ACE_TRACE_IMPL(X)
#  include "ace/Trace.h"
#endif /* (KNSIMMOTOR_NTRACE == 1) */

#endif /* KNSIMMOTOR_EXPORT_H */

// End of auto generated file.
