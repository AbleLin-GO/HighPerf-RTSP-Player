// #pragma once

// #include "AVRAII.h" 
// #include <atomic>
// #include <condition_variable>
// #include <mutex>
// #include <queue>
// #include <string>
// #include <thread>

// // 码率控制模式
// enum class RateControlMode { CRF, CBR_VBV, ABR };

// class VideoTranscoder {
// public:
//   VideoTranscoder();
//   ~VideoTranscoder();

//   // 初始化：配置编码参数
//   bool Init(const std::string &output_file, int width, int height, int fps,
//             RateControlMode mode, int bitrate_kbps = 2000);

//   void Start();
//   void Stop();

//   // 数据输入：接收解码后的原始帧
//   void PushFrame(const AVFrame *src_frame);

// private:
//   void EncodeThreadFunc();
//   void ConfigureRateControl(AVCodecContext *ctx, RateControlMode mode,
//                             int bitrate);

// private:
//   // --- RAII 资源管理 ---
//   // 使用针对输出的 FormatContext
//   UniqueAVOutputFormatContext fmt_ctx_;
//   UniqueAVCodecContext enc_ctx_;

//   // Stream 不由 unique_ptr 管理，它属于 fmt_ctx_
//   AVStream *out_stream_ = nullptr;

//   // --- 线程与同步 ---
//   std::thread encode_thread_;
//   std::atomic<bool> running_{false};
//   std::atomic<bool> finish_signal_{false};

//   // --- 任务队列 ---
//   // 核心：队列中存储的是 UniqueAVFrame，利用移动语义(std::move)传递所有权
//   // 这样队列销毁时，里面的 AVFrame 也会自动释放，零泄露
//   std::queue<UniqueAVFrame> frame_queue_;
//   std::mutex queue_mutex_;
//   std::condition_variable queue_cv_;

//   int64_t pts_counter_ = 0;
// };
#pragma once
#include "AVRAII.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>


extern "C" {
#include <libavutil/audio_fifo.h> // 音频 FIFO
}
// 码率控制模式
enum class RateControlMode { CRF, CBR_VBV, ABR };

class VideoTranscoder {
public:
  VideoTranscoder();
  ~VideoTranscoder();

  // audio_channels: 声道数, audio_rate: 采样率, audio_layout: 声道布局
  bool Init(const std::string &output_file, int width, int height, int fps,
            int audio_channels, int audio_rate, AVChannelLayout audio_layout,
            RateControlMode mode, int bitrate_kbps = 2000);

  void Start();
  void Stop();

  void PushFrame(const AVFrame *src_frame);
  void PushAudio(const AVFrame *src_audio);

private:
  void EncodeThreadFunc();
  void ConfigureRateControl(AVCodecContext *ctx, RateControlMode mode,
                            int bitrate);

  // 内部音频处理函数
  bool InitAudio(int channels, int rate, AVChannelLayout layout);
  void ProcessAudioPacket(AVFrame *frame);
  void FlushAudio();

private:
  // --- 视频资源 ---
  UniqueAVOutputFormatContext fmt_ctx_;
  UniqueAVCodecContext enc_ctx_; // 视频编码器
  AVStream *out_stream_ = nullptr;

  // ---  音频资源 ---
  UniqueAVCodecContext audio_enc_ctx_; // 音频编码器
  AVStream *audio_out_stream_ = nullptr;
  UniqueSwrContext swr_ctx_; // 音频重采样 (格式转换)
  AVAudioFifo *audio_fifo_ =
      nullptr; // 音频缓冲 FIFO (C原生指针，需手动释放)

  // 记录音频源参数，用于检测格式变化
  int src_sample_rate_ = 0;
  AVChannelLayout src_ch_layout_ = {};
  enum AVSampleFormat src_sample_fmt_ = AV_SAMPLE_FMT_NONE;

  // --- 线程与同步 ---
  std::thread encode_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> finish_signal_{false};

  // --- 队列 ---
  std::queue<UniqueAVFrame> video_queue_; // 视频队列
  std::queue<UniqueAVFrame> audio_queue_; //  音频队列

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  int64_t video_pts_ctr_ = 0;
  int64_t audio_pts_ctr_ = 0;
};
