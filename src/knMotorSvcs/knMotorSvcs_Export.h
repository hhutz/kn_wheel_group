
// -*- C++ -*-
// $Id$
// Definition for Win32 Export directives.
// This file is generated automatically by generate_export_file.pl knMotorSvcs
// ------------------------------
#ifndef KNMOTORSVCS_EXPORT_H
#define KNMOTORSVCS_EXPORT_H

#include "ace/config-all.h"

#if defined (ACE_AS_STATIC_LIBS) && !defined (KNMOTORSVCS_HAS_DLL)
#  define KNMOTORSVCS_HAS_DLL 0
#endif /* ACE_AS_STATIC_LIBS && KNMOTORSVCS_HAS_DLL */

#if !defined (KNMOTORSVCS_HAS_DLL)
#  define KNMOTORSVCS_HAS_DLL 1
#endif /* ! KNMOTORSVCS_HAS_DLL */

#if defined (KNMOTORSVCS_HAS_DLL) && (KNMOTORSVCS_HAS_DLL == 1)
#  if defined (KNMOTORSVCS_BUILD_DLL)
#    define knMotorSvcs_Export ACE_Proper_Export_Flag
#    define KNMOTORSVCS_SINGLETON_DECLARATION(T) ACE_EXPORT_SINGLETON_DECLARATION (T)
#    define KNMOTORSVCS_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_EXPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  else /* KNMOTORSVCS_BUILD_DLL */
#    define knMotorSvcs_Export ACE_Proper_Import_Flag
#    define KNMOTORSVCS_SINGLETON_DECLARATION(T) ACE_IMPORT_SINGLETON_DECLARATION (T)
#    define KNMOTORSVCS_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_IMPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  endif /* KNMOTORSVCS_BUILD_DLL */
#else /* KNMOTORSVCS_HAS_DLL == 1 */
#  define knMotorSvcs_Export
#  define KNMOTORSVCS_SINGLETON_DECLARATION(T)
#  define KNMOTORSVCS_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#endif /* KNMOTORSVCS_HAS_DLL == 1 */

// Set KNMOTORSVCS_NTRACE = 0 to turn on library specific tracing even if
// tracing is turned off for ACE.
#if !defined (KNMOTORSVCS_NTRACE)
#  if (ACE_NTRACE == 1)
#    define KNMOTORSVCS_NTRACE 1
#  else /* (ACE_NTRACE == 1) */
#    define KNMOTORSVCS_NTRACE 0
#  endif /* (ACE_NTRACE == 1) */
#endif /* !KNMOTORSVCS_NTRACE */

#if (KNMOTORSVCS_NTRACE == 1)
#  define KNMOTORSVCS_TRACE(X)
#else /* (KNMOTORSVCS_NTRACE == 1) */
#  if !defined (ACE_HAS_TRACE)
#    define ACE_HAS_TRACE
#  endif /* ACE_HAS_TRACE */
#  define KNMOTORSVCS_TRACE(X) ACE_TRACE_IMPL(X)
#  include "ace/Trace.h"
#endif /* (KNMOTORSVCS_NTRACE == 1) */

#endif /* KNMOTORSVCS_EXPORT_H */

// End of auto generated file.
