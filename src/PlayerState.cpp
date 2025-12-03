#include "../includes/PlayerState.h"
#include <iostream>

PlayerState::PlayerState(AVFormatContext *fmt_ctx)
    : ic(fmt_ctx), status(PlayerStatus::STOPPED), quit(false),
      seek_request(false), seek_pos(0), seek_flags(0),
      // 【优化 1】增大 Packet 队列容量
      // 网络流会有突发流量（Burst），100个包可能瞬间被填满导致丢包。
      // 建议设为 500 或更多，给网络抖动留出缓冲空间。
      video_packet_queue(500), video_frame_queue(16), audio_packet_queue(500),
      audio_frame_queue(32), is_exporting(false) {
  // 【优化 2】初始化为立体声 (2通道)
  // 0 通道可能导致 swr_alloc 在第一帧到来前参数校验失败
  av_channel_layout_default(&swr_in_ch_layout, 2);

  // 【优化 3】增大音频环形缓冲区 (Ring Buffer)
  // 你原来的 384000 (约375KB) 对应 48k立体声浮点数约 1秒。
  // 对于网络流，建议给大一点（比如 4MB），防止 "Underrun"（缓冲区耗尽）。
  // 4MB 内存占用很小，但能极大提升抗抖动能力。
  audio_ring_buffer.resize(4 * 1024 * 1024, 0);

  // 显式重置读写指针 (虽然类内有默认值，但在构造函数里写出来更安心)
  audio_buf_size = 0;
  audio_buf_read_pos = 0;
  audio_buf_write_pos = 0;

  std::cout << "[PlayerState] Initialized. Audio RingBuffer: "
            << audio_ring_buffer.size() / 1024 << " KB\n";
}

PlayerState::~PlayerState() {
  // 1. 安全停止转码器
  if (transcoder) {
    std::cout << "[PlayerState] Stopping Transcoder...\n";
    transcoder->Stop();
    transcoder = nullptr;
  }

  // 2. 停止所有队列 (唤醒所有卡在 pop/push 的线程)
  video_packet_queue.abort();
  video_frame_queue.abort();
  audio_packet_queue.abort();
  audio_frame_queue.abort();

  // 3. 释放 FFmpeg 手动管理的资源
  av_channel_layout_uninit(&swr_in_ch_layout);
  av_channel_layout_uninit(&audio_tgt.channel_layout_struct);
  av_channel_layout_uninit(&audio_src.channel_layout_struct);

  std::cout << "[PlayerState] Destroyed.\n";
}