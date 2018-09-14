
// -*- C++ -*-
// $Id$
// Definition for Win32 Export directives.
// This file is generated automatically by generate_export_file.pl knProtoMotor
// ------------------------------
#ifndef KNPROTOMOTOR_EXPORT_H
#define KNPROTOMOTOR_EXPORT_H

#include "ace/config-all.h"

#if defined (ACE_AS_STATIC_LIBS) && !defined (KNPROTOMOTOR_HAS_DLL)
#  define KNPROTOMOTOR_HAS_DLL 0
#endif /* ACE_AS_STATIC_LIBS && KNPROTOMOTOR_HAS_DLL */

#if !defined (KNPROTOMOTOR_HAS_DLL)
#  define KNPROTOMOTOR_HAS_DLL 1
#endif /* ! KNPROTOMOTOR_HAS_DLL */

#if defined (KNPROTOMOTOR_HAS_DLL) && (KNPROTOMOTOR_HAS_DLL == 1)
#  if defined (KNPROTOMOTOR_BUILD_DLL)
#    define knProtoMotor_Export ACE_Proper_Export_Flag
#    define KNPROTOMOTOR_SINGLETON_DECLARATION(T) ACE_EXPORT_SINGLETON_DECLARATION (T)
#    define KNPROTOMOTOR_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_EXPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  else /* KNPROTOMOTOR_BUILD_DLL */
#    define knProtoMotor_Export ACE_Proper_Import_Flag
#    define KNPROTOMOTOR_SINGLETON_DECLARATION(T) ACE_IMPORT_SINGLETON_DECLARATION (T)
#    define KNPROTOMOTOR_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_IMPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  endif /* KNPROTOMOTOR_BUILD_DLL */
#else /* KNPROTOMOTOR_HAS_DLL == 1 */
#  define knProtoMotor_Export
#  define KNPROTOMOTOR_SINGLETON_DECLARATION(T)
#  define KNPROTOMOTOR_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#endif /* KNPROTOMOTOR_HAS_DLL == 1 */

// Set KNPROTOMOTOR_NTRACE = 0 to turn on library specific tracing even if
// tracing is turned off for ACE.
#if !defined (KNPROTOMOTOR_NTRACE)
#  if (ACE_NTRACE == 1)
#    define KNPROTOMOTOR_NTRACE 1
#  else /* (ACE_NTRACE == 1) */
#    define KNPROTOMOTOR_NTRACE 0
#  endif /* (ACE_NTRACE == 1) */
#endif /* !KNPROTOMOTOR_NTRACE */

#if (KNPROTOMOTOR_NTRACE == 1)
#  define KNPROTOMOTOR_TRACE(X)
#else /* (KNPROTOMOTOR_NTRACE == 1) */
#  if !defined (ACE_HAS_TRACE)
#    define ACE_HAS_TRACE
#  endif /* ACE_HAS_TRACE */
#  define KNPROTOMOTOR_TRACE(X) ACE_TRACE_IMPL(X)
#  include "ace/Trace.h"
#endif /* (KNPROTOMOTOR_NTRACE == 1) */

#endif /* KNPROTOMOTOR_EXPORT_H */

// End of auto generated file.
