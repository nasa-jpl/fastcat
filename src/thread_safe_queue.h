#ifndef FASTCAT_THREAD_SAFE_QUEUE_H_
#define FASTCAT_THREAD_SAFE_QUEUE_H_

#include <queue>
#include <mutex>

namespace fastcat {
template <class T>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(void) {}

  ~ThreadSafeQueue(void) {}

  void push(T t)
  {
    std::lock_guard<std::mutex> lock(m);
    q.push(t);
  }

  T front(void)
  {
    std::lock_guard<std::mutex> lock(m);
  	return q.front();  
  }

  void pop(void)
  {
    std::lock_guard<std::mutex> lock(m);
		q.pop();
  }

  bool empty(void)
  {
    std::lock_guard<std::mutex> lock(m);
		return q.empty();
  }

  bool try_pop(T& item)
  {
    std::lock_guard<std::mutex> lock(m);
    if (q.empty()) {
      return false;
    }
    item = q.front();
    q.pop();
    return true;
  }
  

private:
  std::queue<T> q;
  mutable std::mutex m;

};

}
#endif
