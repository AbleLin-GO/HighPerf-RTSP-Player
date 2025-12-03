#include "../includes/PacketQueue.h"

// PacketQueue 构造函数
PacketQueue::PacketQueue(size_t max_size)
    : m_abort_request(false), m_max_size(max_size) {}

// PacketQueue 析构函数
PacketQueue::~PacketQueue() { abort(); }

// 线程安全、阻塞式存入 (Push)
void PacketQueue::push(UniqueAVPacket packet) {
  // 这里必须用 unique_lock，因为 wait 需要解锁等待
  std::unique_lock<std::mutex> lock(m_mutex);

  if (m_abort_request) {
    return;
  }

  // 如果队列满了，阻塞等待消费者取走数据
  // 防止读取速度太快导致内存爆炸
  m_cond.wait(
      lock, [this] { return m_abort_request || m_queue.size() < m_max_size; });

  if (m_abort_request) {
    return;
  }

  // 所有权转移
  m_queue.push(std::move(packet));

  // 通知等待中的线程 (可能是等待数据的 pop，也可能是等待空位的push）
  m_cond.notify_all();
}

// 线程安全且阻塞式取出 (Pop)
bool PacketQueue::pop(UniqueAVPacket &packet_out) {
  std::unique_lock<std::mutex> lock(m_mutex);

  // 阻塞等待：直到队列不空或收到停止请求
  m_cond.wait(lock, [this] { return !m_queue.empty() || m_abort_request; });

  if (m_abort_request) {
    return false;
  }

  // 安全取出数据 (所有权转移)
  packet_out = std::move(m_queue.front());
  m_queue.pop();

  // 取出数据后，队列有了空位，通知可能正在阻塞等待 push 的生产者
  m_cond.notify_all();

  return true;
}

// 停止队列
void PacketQueue::abort() {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_abort_request = true;
    // 清空队列，RAII 自动释放资源
    std::queue<UniqueAVPacket> empty_queue;
    m_queue.swap(empty_queue);
  }
  // 唤醒所有等待线程 (包括正在 wait 的 push 和 pop)
  m_cond.notify_all();
}

// 清空队列
void PacketQueue::flush() {

  std::lock_guard<std::mutex> lock(m_mutex);
  std::queue<UniqueAVPacket> empty_queue;
  m_queue.swap(empty_queue);
  // 唤醒所有等待线程 (包括正在 wait 的 push 和 pop)
  m_cond.notify_all();
}

// 获取大小
size_t PacketQueue::size() {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_queue.size();
}
