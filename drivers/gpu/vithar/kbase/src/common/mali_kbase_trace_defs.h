/*
 *
 * (C) COPYRIGHT 2011-2012 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * 
 */




/* ***** IMPORTANT: THIS IS NOT A NORMAL HEADER FILE         *****
 * *****            DO NOT INCLUDE DIRECTLY                  *****
 * *****            THE LACK OF HEADER GUARDS IS INTENTIONAL ***** */

/*
 * The purpose of this header file is just to contain a list of trace code idenitifers
 *
 * Each identifier is wrapped in a macro, so that its string form and enum form can be created
 *
 * Each macro is separated with a comma, to allow insertion into an array initializer or enum definition block.
 *
 * This allows automatic creation of an enum and a corresponding array of strings
 *
 * Before #including, the includer MUST #define KBASE_TRACE_CODE_MAKE_CODE.
 * After #including, the includer MUST #under KBASE_TRACE_CODE_MAKE_CODE.
 *
 * e.g.:
 * #define KBASE_TRACE_CODE( X ) KBASE_TRACE_CODE_ ## X
 * typedef enum
 * {
 * #define KBASE_TRACE_CODE_MAKE_CODE( X ) KBASE_TRACE_CODE( X )
 * #include "mali_kbase_trace_defs.h"
 * #undef  KBASE_TRACE_CODE_MAKE_CODE
 * } kbase_trace_code;
 *
 * IMPORTANT: THIS FILE MUST NOT BE USED FOR ANY OTHER PURPOSE OTHER THAN THE ABOVE
 *
 *
 * The use of the macro here is:
 * - KBASE_TRACE_CODE_MAKE_CODE( X )
 *
 * Which produces:
 * - For an enum, KBASE_TRACE_CODE_X
 * - For a string, "X"
 *
 *
 * For example:
 * - KBASE_TRACE_CODE_MAKE_CODE( JM_JOB_COMPLETE ) expands to:
 *  - KBASE_TRACE_CODE_JM_JOB_COMPLETE for the enum
 *  - "JM_JOB_COMPLETE" for the string
 * - To use it to trace an event, do:
 *  - KBASE_TRACE_ADD( kbdev, JM_JOB_COMPLETE, subcode, kctx, uatom, val );
 */

/*
 * Core events
 */
KBASE_TRACE_CODE_MAKE_CODE( CORE_CTX_DESTROY ),      /* no info_val, no gpu_addr, no atom */
KBASE_TRACE_CODE_MAKE_CODE( CORE_CTX_HWINSTR_TERM ), /* no info_val, no gpu_addr, no atom */
KBASE_TRACE_CODE_MAKE_CODE( CORE_GPU_IRQ ), /* info_val == GPU_IRQ_STATUS register */

/*
 * Job Slot management events
 */

KBASE_TRACE_CODE_MAKE_CODE( JM_IRQ ),            /* info_val==irq rawstat at start */
KBASE_TRACE_CODE_MAKE_CODE( JM_IRQ_END ),        /* info_val==jobs processed */
/* In the following:
 *
 * - ctx is set if a corresponding job found (NULL otherwise, e.g. some soft-stop cases)
 * - uatom==kernel-side mapped uatom address (for correlation with user-side)
 */
KBASE_TRACE_CODE_MAKE_CODE( JM_JOB_DONE ),       /* info_val==exit code; gpu_addr==chain gpuaddr */
KBASE_TRACE_CODE_MAKE_CODE( JM_SUBMIT ),         /* gpu_addr==JSn_HEAD_NEXT written, info_val==lower 32 bits of affinity */

/* gpu_addr is as follows:
 * - If JSn_STATUS active after soft-stop, val==gpu addr written to JSn_HEAD on submit
 * - otherwise gpu_addr==0 */
KBASE_TRACE_CODE_MAKE_CODE( JM_SOFTSTOP ),
KBASE_TRACE_CODE_MAKE_CODE( JM_HARDSTOP ),       /* gpu_addr==JSn_HEAD read */

KBASE_TRACE_CODE_MAKE_CODE( JM_UPDATE_HEAD ),    /* gpu_addr==JSn_TAIL read */
/* gpu_addr is as follows:
 * - If JSn_STATUS active before soft-stop, val==JSn_HEAD
 * - otherwise gpu_addr==0 */
KBASE_TRACE_CODE_MAKE_CODE( JM_CHECK_HEAD ),     /* gpu_addr==JSn_HEAD read */

KBASE_TRACE_CODE_MAKE_CODE( JM_FLUSH_WORKQS ),
KBASE_TRACE_CODE_MAKE_CODE( JM_FLUSH_WORKQS_DONE ),

KBASE_TRACE_CODE_MAKE_CODE( JM_ZAP_NON_SCHEDULED ), /* info_val == is_scheduled */
KBASE_TRACE_CODE_MAKE_CODE( JM_ZAP_SCHEDULED ),     /* info_val == is_scheduled */
KBASE_TRACE_CODE_MAKE_CODE( JM_ZAP_DONE ),

KBASE_TRACE_CODE_MAKE_CODE( JM_SLOT_SOFT_OR_HARD_STOP ),  /* info_val == nr jobs submitted */
KBASE_TRACE_CODE_MAKE_CODE( JM_SLOT_EVICT ), /* gpu_addr==JSn_HEAD_NEXT last written */
KBASE_TRACE_CODE_MAKE_CODE( JM_SUBMIT_AFTER_RESET ),


/*
 * Job dispatch events
 */
KBASE_TRACE_CODE_MAKE_CODE( JD_DONE ),     /* gpu_addr==value to write into JSn_HEAD */
KBASE_TRACE_CODE_MAKE_CODE( JD_DONE_WORKER ),     /* gpu_addr==value to write into JSn_HEAD */
KBASE_TRACE_CODE_MAKE_CODE( JD_DONE_WORKER_END ), /* gpu_addr==value to write into JSn_HEAD */
KBASE_TRACE_CODE_MAKE_CODE( JD_DONE_TRY_RUN_NEXT_JOB ), /* gpu_addr==value to write into JSn_HEAD */

KBASE_TRACE_CODE_MAKE_CODE( JD_ZAP_CONTEXT ),     /* gpu_addr==0, info_val==0, uatom==0 */
KBASE_TRACE_CODE_MAKE_CODE( JD_CANCEL ),     /* gpu_addr==value to write into JSn_HEAD */
KBASE_TRACE_CODE_MAKE_CODE( JD_CANCEL_WORKER ),     /* gpu_addr==value to write into JSn_HEAD */

/*
 * Scheduler Core events
 */

KBASE_TRACE_CODE_MAKE_CODE( JS_RETAIN_CTX_NOLOCK ),
KBASE_TRACE_CODE_MAKE_CODE( JS_ADD_JOB ),              /* gpu_addr==value to write into JSn_HEAD */
KBASE_TRACE_CODE_MAKE_CODE( JS_REMOVE_JOB ),           /* gpu_addr==last value written/would be written to JSn_HEAD */
KBASE_TRACE_CODE_MAKE_CODE( JS_RETAIN_CTX ),
KBASE_TRACE_CODE_MAKE_CODE( JS_RELEASE_CTX ),
KBASE_TRACE_CODE_MAKE_CODE( JS_TRY_SCHEDULE_HEAD_CTX ),
KBASE_TRACE_CODE_MAKE_CODE( JS_JOB_DONE_TRY_RUN_NEXT_JOB ), /* gpu_addr==value to write into JSn_HEAD */
KBASE_TRACE_CODE_MAKE_CODE( JS_JOB_DONE_RETRY_NEEDED ), /* gpu_addr==value to write into JSn_HEAD */
KBASE_TRACE_CODE_MAKE_CODE( JS_FAST_START_EVICTS_CTX ), /* kctx is the one being evicted, info_val == kctx to put in  */
KBASE_TRACE_CODE_MAKE_CODE( JS_AFFINITY_SUBMIT_TO_BLOCKED ),
KBASE_TRACE_CODE_MAKE_CODE( JS_AFFINITY_CURRENT ), /* info_val == lower 32 bits of affinity */
KBASE_TRACE_CODE_MAKE_CODE( JS_CORE_REF_REQUEST_CORES_FAILED ), /* info_val == lower 32 bits of affinity */
KBASE_TRACE_CODE_MAKE_CODE( JS_CORE_REF_REGISTER_INUSE_FAILED ), /* info_val == lower 32 bits of affinity */
KBASE_TRACE_CODE_MAKE_CODE( JS_CORE_REF_REQUEST_ON_RECHECK_FAILED ), /* info_val == lower 32 bits of rechecked affinity */
KBASE_TRACE_CODE_MAKE_CODE( JS_CORE_REF_REGISTER_ON_RECHECK_FAILED ), /* info_val == lower 32 bits of rechecked affinity */
KBASE_TRACE_CODE_MAKE_CODE( JS_CORE_REF_AFFINITY_WOULD_VIOLATE ), /* info_val == lower 32 bits of affinity */
KBASE_TRACE_CODE_MAKE_CODE( JS_CTX_ATTR_NOW_ON_CTX ), /* info_val == the ctx attribute now on ctx */
KBASE_TRACE_CODE_MAKE_CODE( JS_CTX_ATTR_NOW_ON_RUNPOOL ), /* info_val == the ctx attribute now on runpool */
KBASE_TRACE_CODE_MAKE_CODE( JS_CTX_ATTR_NOW_OFF_CTX ), /* info_val == the ctx attribute now off ctx */
KBASE_TRACE_CODE_MAKE_CODE( JS_CTX_ATTR_NOW_OFF_RUNPOOL ), /* info_val == the ctx attribute now off runpool */


/*
 * Scheduler Policy events
 */

KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_INIT_CTX ),
KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_TERM_CTX ),
KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_TRY_EVICT_CTX ), /* info_val == whether it was evicted */
KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_KILL_ALL_CTX_JOBS ),
KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_ENQUEUE_CTX ),
KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_DEQUEUE_HEAD_CTX ),
KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_RUNPOOL_ADD_CTX ),
KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_RUNPOOL_REMOVE_CTX ),

KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_DEQUEUE_JOB ),
KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_DEQUEUE_JOB_IRQ ),
KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_ENQUEUE_JOB ),     /* gpu_addr==JSn_HEAD to write if the job were run */
KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_TIMER_START ),
KBASE_TRACE_CODE_MAKE_CODE( JS_POLICY_TIMER_END ),


/*
 * Power Management Events
 */
KBASE_TRACE_CODE_MAKE_CODE( PM_JOB_SUBMIT_AFTER_POWERING_UP ),
KBASE_TRACE_CODE_MAKE_CODE( PM_JOB_SUBMIT_AFTER_POWERED_UP ),
KBASE_TRACE_CODE_MAKE_CODE( PM_PWRON ),
KBASE_TRACE_CODE_MAKE_CODE( PM_PWROFF ),
KBASE_TRACE_CODE_MAKE_CODE( PM_CORES_POWERED ),
KBASE_TRACE_CODE_MAKE_CODE( PM_CORES_CHANGE_DESIRED_ON_POWERUP ),
KBASE_TRACE_CODE_MAKE_CODE( PM_CORES_CHANGE_DESIRED_ON_POWERDOWN ),
KBASE_TRACE_CODE_MAKE_CODE( PM_CORES_CHANGE_DESIRED ),
KBASE_TRACE_CODE_MAKE_CODE( PM_CORES_CHANGE_AVAILABLE ),
KBASE_TRACE_CODE_MAKE_CODE( PM_REGISTER_CHANGE_SHADER_INUSE ),
KBASE_TRACE_CODE_MAKE_CODE( PM_REGISTER_CHANGE_SHADER_NEEDED ),
KBASE_TRACE_CODE_MAKE_CODE( PM_RELEASE_CHANGE_SHADER_INUSE ),
KBASE_TRACE_CODE_MAKE_CODE( PM_UNREQUEST_CHANGE_SHADER_NEEDED ),
KBASE_TRACE_CODE_MAKE_CODE( PM_REQUEST_CHANGE_SHADER_NEEDED ),
KBASE_TRACE_CODE_MAKE_CODE( PM_CONTEXT_ACTIVE ),
KBASE_TRACE_CODE_MAKE_CODE( PM_CONTEXT_IDLE ),
KBASE_TRACE_CODE_MAKE_CODE( PM_SEND_EVENT ), /* info_val == event code */
KBASE_TRACE_CODE_MAKE_CODE( PM_HANDLE_EVENT ), /* info_val == event code */
KBASE_TRACE_CODE_MAKE_CODE( PM_ACTIVATE_WORKER ),


/* Unused code just to make it easier to not have a comma at the end.
 * All other codes MUST come before this */
KBASE_TRACE_CODE_MAKE_CODE( DUMMY )

/* ***** THE LACK OF HEADER GUARDS IS INTENTIONAL ***** */
