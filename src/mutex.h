#ifndef FASTCAT_MUTEX_H_
#define FASTCAT_MUTEX_H_

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>

namespace fastcat
{
class Mutex
{
 public:
  Mutex() { k_mutex_init(&mutex_); }

  Mutex(const Mutex&) = delete;
  Mutex& operator=(const Mutex&) = delete;

  void lock() { (void)k_mutex_lock(&mutex_, K_FOREVER); }
  void unlock() { (void)k_mutex_unlock(&mutex_); }

 private:
  struct k_mutex mutex_;
};

template <typename MutexType>
class LockGuard
{
 public:
  explicit LockGuard(MutexType& mutex)
      : mutex_(mutex)
  {
    mutex_.lock();
  }

  LockGuard(const LockGuard&) = delete;
  LockGuard& operator=(const LockGuard&) = delete;

  ~LockGuard() { mutex_.unlock(); }

 private:
  MutexType& mutex_;
};
}  // namespace fastcat

#else
#include <mutex>

namespace fastcat
{
using Mutex = std::mutex;

template <typename MutexType>
using LockGuard = std::lock_guard<MutexType>;
}  // namespace fastcat
#endif

#endif
