#pragma once

#include "AVRAII.h"
#include <condition_variable>
#include <mutex>
#include <queue>

/**
 * @brief 线程安全、阻塞且有界的 AVPacket 队列。
 */
class PacketQueue {
public:
  // 默认限制 100 个包，防止解复用速度过快导致 OOM
  PacketQueue(size_t max_size = 100);
  ~PacketQueue();

  /**
   * @brief 线程安全地向队列中添加 AVPacket。
   * 如果队列已满，此函数会阻塞，直到有空间或队列停止。
   * @param packet 要添加的 AVPacket（所有权转移给队列）。
   */
  void push(UniqueAVPacket packet);

  /**
   * @brief 线程安全且阻塞地从队列中取出 AVPacket。
   * @param packet_out 输出参数，接收取出的 AVPacket。
   * @return 如果成功取出返回 true；如果队列被停止 (abort) 返回 false。
   */
  bool pop(UniqueAVPacket &packet_out);

  void abort();
  size_t size();
  // 【新增】 清空队列
  void flush();

private:
  std::queue<UniqueAVPacket> m_queue; // 存储 UniqueAVPacket
  std::mutex m_mutex;
  std::condition_variable m_cond;
  bool m_abort_request;
  size_t m_max_size; // 最大容量限制
};