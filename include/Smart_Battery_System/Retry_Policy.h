#pragma once

#include <cstdint>

/* Generic bounded retry loop with exponential backoff.
 *
 * Zephyr-free on purpose: sleeping, watchdog feeding and statistics are passed
 * in as callables, so the policy can be unit-tested on a host.
 *
 * Per attempt (0..max_retries inclusive) the order is:
 *   feed() -> read() -> is_ok? return
 *   on_retry() -> (if more attempts remain) sleep(backoff), backoff doubles up to the cap
 * The last response is returned when every attempt failed. */
namespace RetryPolicy {

struct Config {
    uint32_t max_retries;
    uint32_t initial_backoff_ms;
    uint32_t max_backoff_ms;
};

template <typename R, typename ReadFn, typename IsOkFn,
          typename FeedFn, typename OnRetryFn, typename SleepFn>
R run(const Config& cfg, R response, ReadFn read, IsOkFn is_ok,
      FeedFn feed, OnRetryFn on_retry, SleepFn sleep_ms) {
    uint32_t backoff_ms = cfg.initial_backoff_ms;

    for (uint32_t attempt = 0U; attempt <= cfg.max_retries; ++attempt) {
        feed();
        response = read();
        if (is_ok(response)) {
            return response;
        }
        on_retry();
        if (attempt < cfg.max_retries) {
            sleep_ms(backoff_ms);
            backoff_ms = (backoff_ms >= (cfg.max_backoff_ms / 2U)) ? cfg.max_backoff_ms
                                                                   : (backoff_ms * 2U);
        }
    }
    return response;
}

} // namespace RetryPolicy
