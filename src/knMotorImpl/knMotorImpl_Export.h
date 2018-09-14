
// -*- C++ -*-
// $Id$
// Definition for Win32 Export directives.
// This file is generated automatically by generate_export_file.pl knMotor
// ------------------------------
#ifndef KNMOTORIMPL_EXPORT_H
#define KNMOTORIMPL_EXPORT_H

#include "ace/config-all.h"

#if defined (ACE_AS_STATIC_LIBS) && !defined (KNMOTORIMPL_HAS_DLL)
#  define KNMOTORIMPL_HAS_DLL 0
#endif /* ACE_AS_STATIC_LIBS && KNMOTORIMPL_HAS_DLL */

#if !defined (KNMOTORIMPL_HAS_DLL)
#  define KNMOTORIMPL_HAS_DLL 1
#endif /* ! KNMOTORIMPL_HAS_DLL */

#if defined (KNMOTORIMPL_HAS_DLL) && (KNMOTORIMPL_HAS_DLL == 1)
#  if defined (KNMOTORIMPL_BUILD_DLL)
#    define knMotorImpl_Export ACE_Proper_Export_Flag
#    define KNMOTORIMPL_SINGLETON_DECLARATION(T) ACE_EXPORT_SINGLETON_DECLARATION (T)
#    define KNMOTORIMPL_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_EXPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  else /* KNMOTORIMPL_BUILD_DLL */
#    define knMotorImpl_Export ACE_Proper_Import_Flag
#    define KNMOTORIMPL_SINGLETON_DECLARATION(T) ACE_IMPORT_SINGLETON_DECLARATION (T)
#    define KNMOTORIMPL_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_IMPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  endif /* KNMOTORIMPL_BUILD_DLL */
#else /* KNMOTORIMPL_HAS_DLL == 1 */
#  define knMotorImpl_Export
#  define KNMOTORIMPL_SINGLETON_DECLARATION(T)
#  define KNMOTORIMPL_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#endif /* KNMOTORIMPL_HAS_DLL == 1 */

// Set KNMOTORIMPL_NTRACE = 0 to turn on library specific tracing even if
// tracing is turned off for ACE.
#if !defined (KNMOTORIMPL_NTRACE)
#  if (ACE_NTRACE == 1)
#    define KNMOTORIMPL_NTRACE 1
#  else /* (ACE_NTRACE == 1) */
#    define KNMOTORIMPL_NTRACE 0
#  endif /* (ACE_NTRACE == 1) */
#endif /* !KNMOTORIMPL_NTRACE */

#if (KNMOTORIMPL_NTRACE == 1)
#  define KNMOTORIMPL_TRACE(X)
#else /* (KNMOTORIMPL_NTRACE == 1) */
#  if !defined (ACE_HAS_TRACE)
#    define ACE_HAS_TRACE
#  endif /* ACE_HAS_TRACE */
#  define KNMOTORIMPL_TRACE(X) ACE_TRACE_IMPL(X)
#  include "ace/Trace.h"
#endif /* (KNMOTORIMPL_NTRACE == 1) */

#endif /* KNMOTORIMPL_EXPORT_H */

// End of auto generated file.
