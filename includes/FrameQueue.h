#pragma once

#include "AVRAII.h"
#include <condition_variable>
#include <mutex>
#include <queue>

class FrameQueue {
public:
  // Frame 占用内存大（一帧 1080p YUV420P 约 3MB），默认限制 16 帧即可
  FrameQueue(size_t max_size = 16);
  ~FrameQueue();

  void push(UniqueAVFrame frame);
  bool pop(UniqueAVFrame &frame_out);

  void abort();
  size_t size();
  // 清空队列
  void flush();

private:
  std::queue<UniqueAVFrame> m_queue;
  std::mutex m_mutex;
  std::condition_variable m_cond;
  bool m_abort_request;
  size_t m_max_size; // 最大容量限制
};
