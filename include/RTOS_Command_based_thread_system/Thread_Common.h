#pragma once

#include "System_Objects.h"

#ifdef IS_TEST_ENVIRONMENT
    extern int test_iterations_remaining;
    #define THREAD_LOOP_CONDITION (test_iterations_remaining > 0 ? (test_iterations_remaining--, true) : false)
#else
    #define THREAD_LOOP_CONDITION true
#endif
