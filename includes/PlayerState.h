#pragma once

#include "../includes/VideoTranscoder.h"
#include "AVRAII.h"
#include "FrameQueue.h"
#include "PacketQueue.h"
#include <SDL2/SDL.h>
#include <atomic>
#include <mutex>
#include <vector>
extern "C" {
#include <libavutil/channel_layout.h>
#include <libavutil/motion_vector.h> // [关键] 修复 mvs 报错需要这个头文件
#include <libavutil/opt.h>
#include <libavutil/time.h>
}

enum class PlayerStatus { STOPPED, PAUSED, PLAYING, SEEKING };

enum class AVSyncType {
  AV_SYNC_AUDIO_MASTER,
  AV_SYNC_VIDEO_MASTER,
  AV_SYNC_EXTERNAL_MASTER
};

/**
 * @brief PlayerState 管理所有资源、状态和时钟。
 */
class PlayerState {
public:
  // 构造与析构
  PlayerState(AVFormatContext *fmt_ctx);
  ~PlayerState();

  // 禁用拷贝
  PlayerState(const PlayerState &) = delete;
  PlayerState &operator=(const PlayerState &) = delete;

  // --- 全局控制 ---
  UniqueAVFormatContext ic;
  std::atomic<PlayerStatus> status;
  std::atomic<bool> quit;

  // --- Seek 控制 ---
  std::atomic<bool> seek_request;
  std::atomic<int64_t> seek_pos;
  std::atomic<int> seek_flags;
  std::mutex seek_mutex;

  // --- 音视频同步 (AV Sync) ---
  AVSyncType sync_type = AVSyncType::AV_SYNC_AUDIO_MASTER;
  std::atomic<double> audio_clock{0.0};
  double video_clock = 0.0;
  double frame_timer = 0.0;
  double frame_last_delay = 40e-3;
  double frame_last_pts = 0.0;

  // --- 视频管线 ---
  int video_stream_index = -1;
  AVStream *video_stream = nullptr;
  PacketQueue video_packet_queue;
  UniqueAVCodecContext video_codec_ctx;
  FrameQueue video_frame_queue;
  UniqueSwsContext sws_ctx;

  // 视频渲染共享区
  std::mutex video_mutex;
  std::vector<uint8_t> video_buf;
  int video_width = 0;
  int video_height = 0;
  int video_buf_w = 0;
  int video_buf_h = 0;
  int video_linesize = 0;
  bool video_frame_ready = false;

  // [关键修复] 运动矢量 (Motion Vectors)
  // 修复: No member named 'mvs'
  std::vector<AVMotionVector> mvs;
  bool mv_visualization_enabled = true;

  // --- 音频管线 ---
  int audio_stream_index = -1;
  AVStream *audio_stream = nullptr;
  PacketQueue audio_packet_queue;
  UniqueAVCodecContext audio_codec_ctx;
  FrameQueue audio_frame_queue; // 音频帧队列

  // [关键修复] 音频重采样上下文记录 (用于检测格式变化)
  // 修复: No member named 'swr_in_...'
  UniqueSwrContext swr_ctx;
  AVSampleFormat swr_in_fmt = AV_SAMPLE_FMT_NONE;
  int swr_in_rate = 0;
  AVChannelLayout swr_in_ch_layout = {}; // 初始化为空

  // 音频参数结构体
  struct AudioParams {
    int freq;
    int channels;
    int64_t channel_layout;
    // [关键修复] 新版 FFmpeg 声道布局结构体
    // 修复: No member named 'channel_layout_struct'
    AVChannelLayout channel_layout_struct;
    enum AVSampleFormat fmt;
    int frame_size;
    int bytes_per_sec;
  };

  AudioParams audio_tgt = {}; // 目标参数 (SDL)
  AudioParams audio_src = {}; // 源参数 (解码器输出)

  // 音频环形缓冲区 (SDL 回调使用)
  std::mutex audio_mutex;
  std::vector<uint8_t> audio_ring_buffer;
  size_t audio_buf_size = 0;
  size_t audio_buf_read_pos = 0;
  size_t audio_buf_write_pos = 0;

  // PI 控制器相关
  double audio_diff_cum = 0.0; // 积分项 (累积误差)
  double audio_diff_avg_count = 0.0; // 用于计算平均误差（可选，这里用作计数器）

  // ==========================================
  // 【新增 2】视频导出/转码模块
  // ==========================================
  std::shared_ptr<VideoTranscoder> transcoder; // 转码器实例
  std::atomic<bool> is_exporting{false}; // 标记是否正在进行导出任务

  SDL_Window *window = nullptr;
};