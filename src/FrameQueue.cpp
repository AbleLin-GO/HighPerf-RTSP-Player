#include "../includes/FrameQueue.h"

// FrameQueue 构造函数
FrameQueue::FrameQueue(size_t max_size)
    : m_abort_request(false), m_max_size(max_size) {}

// FrameQueue 析构函数
FrameQueue::~FrameQueue() { abort(); }

void FrameQueue::push(UniqueAVFrame frame) {
  // ：改为 unique_lock 以支持 wait
  std::unique_lock<std::mutex> lock(m_mutex);

  if (m_abort_request) {
    return;
  }

  // 如果堆积了太多未渲染的帧，解码线程暂停
  m_cond.wait(
      lock, [this] { return m_abort_request || m_queue.size() < m_max_size; });

  if (m_abort_request) {
    return;
  }

  m_queue.push(std::move(frame));
  m_cond.notify_all(); // 通知 pop
}

bool FrameQueue::pop(UniqueAVFrame &frame_out) {
  std::unique_lock<std::mutex> lock(m_mutex);

  m_cond.wait(lock, [this] { return m_abort_request || !m_queue.empty(); });

  if (m_abort_request) {
    return false;
  }

  frame_out = std::move(m_queue.front());
  m_queue.pop();

  // 取走一帧后，通知可能阻塞的解码线程继续工作
  m_cond.notify_all();

  return true;
}

void FrameQueue::abort() {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_abort_request = true;
    std::queue<UniqueAVFrame> empty_queue;
    m_queue.swap(empty_queue);
  }
  m_cond.notify_all();
}

size_t FrameQueue::size() {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_queue.size();
}

// 清空队列
void FrameQueue::flush() {

  std::lock_guard<std::mutex> lock(m_mutex);
  std::queue<UniqueAVFrame> empty_queue;
  m_queue.swap(empty_queue);
  // 唤醒所有等待线程 (包括正在 wait 的 push 和 pop)
  m_cond.notify_all();
}
