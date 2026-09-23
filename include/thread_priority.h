#ifndef THREAD_PRIORITY_H
#define THREAD_PRIORITY_H

#include <chrono>
#include <cstdint>

#if defined(__APPLE__)
  #include <mach/mach.h>
  #include <mach/mach_time.h>
  #include <mach/thread_policy.h>
  #include <pthread.h>
#elif defined(_WIN32)
  #include <windows.h>
#else
  #include <pthread.h>
  #include <sched.h>
#endif

namespace rt {

// Asks the scheduler to run the calling thread ahead of ordinary threads. The
// thread is expected to wake about once per `period` and need at most
// `computation` of CPU each time; only macOS uses these figures.
//
// Returns false if the platform refused, e.g. without CAP_SYS_NICE on Linux.
// The thread keeps running at its old priority then.
[[nodiscard]] inline bool makeCurrentThreadRealtime(std::chrono::nanoseconds period,
                                                    std::chrono::nanoseconds computation) noexcept {
#if defined(__APPLE__)
    mach_timebase_info_data_t timebase;
    if (mach_timebase_info(&timebase) != KERN_SUCCESS) {
        return false;
    }
    const auto toAbsolute = [&timebase](std::chrono::nanoseconds ns) {
        return static_cast<std::uint32_t>(static_cast<double>(ns.count()) * timebase.denom / timebase.numer);
    };

    thread_time_constraint_policy_data_t policy;
    policy.period = toAbsolute(period);
    policy.computation = toAbsolute(computation);
    policy.constraint = toAbsolute(period);
    policy.preemptible = TRUE;
    return thread_policy_set(pthread_mach_thread_np(pthread_self()), THREAD_TIME_CONSTRAINT_POLICY,
                             reinterpret_cast<thread_policy_t>(&policy),
                             THREAD_TIME_CONSTRAINT_POLICY_COUNT) == KERN_SUCCESS;
#elif defined(_WIN32)
    (void)period;
    (void)computation;
    return SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL) != 0;
#else
    (void)period;
    (void)computation;
    // Midway, so audio threads, which usually run higher, still come first.
    sched_param param{};
    param.sched_priority = (sched_get_priority_min(SCHED_FIFO) + sched_get_priority_max(SCHED_FIFO)) / 2;
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0;
#endif
}

}  // namespace rt

#endif  // THREAD_PRIORITY_H
