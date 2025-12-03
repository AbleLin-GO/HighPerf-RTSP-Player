extern "C" {
#include <SDL2/SDL.h>
#include <libavcodec/avcodec.h>
#include <libavcodec/packet.h>
#include <libavformat/avformat.h>
#include <libavutil/display.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/timestamp.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}
#include "../includes/PlayerState.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector> // 必须包含

#define FF_REFRESH_EVENT (SDL_USEREVENT + 1)

// --- 辅助函数 ---
int find_stream_index(AVFormatContext *fmt_ctx, AVMediaType type) {
  for (int i = 0; i < fmt_ctx->nb_streams; i++) {
    if (fmt_ctx->streams[i]->codecpar->codec_type == type) {
      return i;
    }
  }
  return -1;
}

// --- 1. 读取线程 (RAII版) ---
void read_thread_func(std::shared_ptr<PlayerState> state) {
  std::cout << "[ReadThread] 启动...\n";
  auto packet_raw = make_unique_packet();

  while (!state->quit) {
    // =================================================
    // 【新增】 处理 Seek 请求
    // =================================================
    if (state->seek_request) {
      int64_t seek_target = state->seek_pos;
      int64_t seek_min = INT64_MIN;
      int64_t seek_max = INT64_MAX;

      std::cout << "[ReadThread] Seeking to timestamp: " << seek_target
                << "...\n";

      // 1. 调用 FFmpeg API 进行跳转
      // AVSEEK_FLAG_BACKWARD: 找最近的关键帧 (I帧)
      int ret = avformat_seek_file(state->ic.get(), -1, seek_min, seek_target,
                                   seek_max, AVSEEK_FLAG_BACKWARD);

      if (ret >= 0) {
        // 2. 成功跳转后，放入 Flush 包通知解码器
        if (state->video_stream_index >= 0) {
          state->video_packet_queue.flush();                   // 清空旧包
          state->video_packet_queue.push(make_flush_packet()); // 发送红牌
        }
        if (state->audio_stream_index >= 0) {
          state->audio_packet_queue.flush();                   // 清空旧包
          state->audio_packet_queue.push(make_flush_packet()); // 发送红牌
        }
      } else {
        std::cerr << "[ReadThread] Seek failed!\n";
      }

      state->seek_request = false; // 请求已消费
      continue; // 跳过本次读取，直接进入下一次循环读取新位置数据
    }
    // =================================================

    av_packet_unref(packet_raw.get());

    int ret = av_read_frame(state->ic.get(), packet_raw.get());
    if (ret < 0) {
      if (ret == AVERROR_EOF) {
        if (state->video_stream_index >= 0)
          state->video_packet_queue.push(make_unique_packet()); // 空包作为 EOF
        if (state->audio_stream_index >= 0)
          state->audio_packet_queue.push(make_unique_packet());
        std::cout << "[ReadThread] EOF reached.\n";
      }
      break;
    }

    if (packet_raw->stream_index == state->video_stream_index) {
      auto pkt = make_unique_packet();
      av_packet_move_ref(pkt.get(), packet_raw.get());
      state->video_packet_queue.push(std::move(pkt));
    } else if (packet_raw->stream_index == state->audio_stream_index) {
      auto pkt = make_unique_packet();
      av_packet_move_ref(pkt.get(), packet_raw.get());
      state->audio_packet_queue.push(std::move(pkt));
    }
  }
  std::cout << "[ReadThread] 退出...\n";
}

// --- 2. 视频解码线程 ---
void video_decode_thread_func(std::shared_ptr<PlayerState> state) {
  std::cout << "[VideoDecodeThread] 启动...\n";
  auto frame_raw = make_unique_frame();
  UniqueAVPacket packet;

  while (!state->quit) {
    if (!state->video_packet_queue.pop(packet))
      break;
    // =================================================
    // 【新增】 响应 Flush 包
    // =================================================
    if (packet->stream_index == -1) {
      // 1. 冲刷解码器内部缓存 (非常重要，否则画面会花)
      avcodec_flush_buffers(state->video_codec_ctx.get());

      // 2. 清空渲染队列 (旧的画面不要了)
      state->video_frame_queue.flush();

      // 3. 重置同步时钟状态 (防止 PI 控制器因为时间跳变而发疯)
      state->frame_last_pts = 0.0;
      state->frame_timer = 0.0;
      state->frame_last_delay = 40e-3;

      std::cout << "[Video] Flushed & Reset!\n";
      continue; // 消费完 Flush 包，继续处理下一个新包
    }
    // =================================================

    if (packet->data == nullptr) {
      avcodec_send_packet(state->video_codec_ctx.get(), nullptr); // Flush
    } else {
      if (avcodec_send_packet(state->video_codec_ctx.get(), packet.get()) < 0)
        continue;
    }

    while (true) {
      int ret =
          avcodec_receive_frame(state->video_codec_ctx.get(), frame_raw.get());
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        break;
      if (ret < 0) {
        state->quit = true;
        break;
      }

      // 设置 PTS
      if (frame_raw->pts == AV_NOPTS_VALUE)
        frame_raw->pts = frame_raw->best_effort_timestamp;

      // 提取 MV (Motion Vectors)
      if (state->mv_visualization_enabled) {
        AVFrameSideData *sd = av_frame_get_side_data(
            frame_raw.get(), AV_FRAME_DATA_MOTION_VECTORS);
        if (sd) {
          std::lock_guard<std::mutex> lock(state->video_mutex);
          const AVMotionVector *mvs = (const AVMotionVector *)sd->data;
          int mv_count = sd->size / sizeof(AVMotionVector);
          state->mvs.assign(mvs, mvs + mv_count);
        }
      }

      // =======================================================
      // 【新增】 视频导出分流逻辑 (Side-Channel)
      // =======================================================
      if (state->is_exporting && state->transcoder) {
        // 将解码出来的原始 YUV 数据推送到转码器队列
        // PushFrame 内部会进行深拷贝，所以不影响下面的播放逻辑
        state->transcoder->PushFrame(frame_raw.get());
      }
      // =======================================================

      // 深拷贝 Frame 推入队列 (与 clone 相比，move_ref 更高效且安全)
      auto frame_to_push = make_unique_frame();
      av_frame_move_ref(frame_to_push.get(), frame_raw.get());
      state->video_frame_queue.push(std::move(frame_to_push));
    }
    if (packet->data == nullptr)
      break;
  }
  state->video_frame_queue.push(UniqueAVFrame(nullptr)); // EOF
  std::cout << "[VideoDecodeThread] 退出...\n";
}

// ============================================================================
// 3. 视频处理/定时线程 (内存安全修复版 + 逻辑修复版)
// ============================================================================
// --- 3. 视频显示线程 (修复开头丢失版) ---
void video_display_thread_func(std::shared_ptr<PlayerState> state) {
  std::cout << "[VideoDisplayThread] 启动...\n";

  UniqueAVFrame frame;
  auto frame_rgb = make_unique_frame();
  std::vector<uint8_t> local_rgb_buffer;
  int dst_w = 0, dst_h = 0;

  // --- 同步阈值常量 (参考 FFplay) ---
  const double AV_SYNC_THRESHOLD_MIN = 0.04; // 最小同步阈值 40ms
  const double AV_SYNC_THRESHOLD_MAX = 0.1;  // 最大同步阈值 100ms
  // --- PI 控制器参数（核心调优参数）---
  const double KP = 0.8;           // 比例增益 (P: Proportional)
  const double KI = 0.005;         // 积分增益 (I: Integral)
  const double MAX_INTEGRAL = 1.0; // 积分项最大限制，防止积分饱和

  // 【新增】连续丢帧计数器
  int drop_count = 0;

  while (!state->quit) {
    frame.reset();
    if (state->status == PlayerStatus::PAUSED) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    if (!state->video_frame_queue.pop(frame))
      break;
    if (!frame) {
      break;
    }

    // --- 同步逻辑开始 ---
    if (frame->pts != AV_NOPTS_VALUE) {
      double video_clock = frame->pts * av_q2d(state->video_stream->time_base);

      // 1. 计算当前帧的持续时间 (Duration)
      double frame_duration = 0.0;
      if (state->frame_last_pts != 0.0) {
        frame_duration = video_clock - state->frame_last_pts;
      }
      if (frame_duration <= 0 || frame_duration > 10.0) {
        frame_duration = state->frame_last_delay;
      }

      // 更新“上一帧”状态
      state->frame_last_pts = video_clock;
      state->frame_last_delay = frame_duration;

      // 2. 获取精确的音频时钟 (主时钟)
      double audio_clock = state->audio_clock;
      double buffered_duration = 0.0;
      {
        std::lock_guard<std::mutex> lock(state->audio_mutex);
        if (state->audio_tgt.bytes_per_sec > 0) {
          // 计算缓冲区内还剩多少时间的数据待播放
          buffered_duration =
              (double)state->audio_buf_size / state->audio_tgt.bytes_per_sec;
        }
      }
      // 声卡当前播放的实际时间 = 帧PTS - 缓冲区延迟
      double current_audio_time = audio_clock - buffered_duration;

      // 3. 计算时间差 (误差)
      double diff = video_clock - current_audio_time;

      // -------------------------------------------------------------
      //  核心修改：应用 PI 控制器思想动态调整延时
      // -------------------------------------------------------------
      double delay = frame_duration; // 默认延时为帧持续时间

      // --- PI 控制逻辑 ---

      // P 项：比例调整，直接反应当前误差
      double p_adjust = KP * diff;

      // I 项：积分累积，消除长期小误差
      // 累积误差，并限制其最大值（防止积分饱和）
      state->audio_diff_cum += diff * frame_duration;
      state->audio_diff_cum = std::min(
          MAX_INTEGRAL, std::max(-MAX_INTEGRAL, state->audio_diff_cum));

      double i_adjust = KI * state->audio_diff_cum;

      // 总校正量
      double total_adjust = p_adjust + i_adjust;

      // 4. 应用校正量到延时 (Delay)
      delay += total_adjust;

      // --- 强制边界和丢帧/跳帧策略 ---

      // 1) 严重滞后（落后超过两帧时间）：必须丢帧追赶
      if (diff < -2.0 * frame_duration) {
        delay = 0; // 立即播放，下一帧再继续追
        std::cout << "[VideoSync] 严重落后 (" << diff * 1000.0
                  << "ms), 强制丢弃当前帧.\n";
        // 不执行渲染，直接跳到下一次循环读取下一帧
        av_frame_unref(frame.get());
        continue;
      }

      // 2) 超前/滞后幅度限制：保证 delay 在合理范围内
      // 限制 delay 在 [0.5 * frame_duration, 2.0 * frame_duration]
      delay = std::min(delay, 2.0 * frame_duration);
      delay = std::max(delay, frame_duration * 0.5);

      // -------------------------------------------------------------

      // 5. 最终休眠控制 (使用 frame_timer 消除累积误差)
      double time = (double)av_gettime_relative() / 1000000.0;

      if (state->frame_timer == 0.0) {
        state->frame_timer = time;
      }

      // 目标：下一帧应该在 state->frame_timer + delay 时刻结束
      double remaining_time = state->frame_timer + delay - time;

      // 如果已经过期，不休眠
      if (remaining_time <= 0) {
        remaining_time = 0;
      }

      // 执行休眠
      if (remaining_time > 0) {
        av_usleep((unsigned)(remaining_time * 1000000.0));
      }

      // 更新 frame_timer：用当前系统时间 + 剩余休息时间，消除操作系统调度误差
      state->frame_timer = time + remaining_time;

      //(可选的日志打印，帮助你调优 PI 参数)
      std::cout << "[VideoSync] V_CLK: " << std::fixed << video_clock
                << " | A_CLK: " << std::fixed << current_audio_time
                << " | DIFF: " << std::setprecision(1) << diff * 1000.0 << "ms"
                << " | DELAY: " << std::setprecision(1) << delay * 1000.0
                << "ms" << " | Remain: " << std::setprecision(1)
                << remaining_time * 1000.0 << "ms\n";
    }
    // --- 同步逻辑结束，继续渲染 ---

    // 3. 渲染 (保持不变，注意对齐)
    if (frame->width > 0 && frame->height > 0) {
      // ... (同之前的 sws_getContext 代码) ...
      if (!state->sws_ctx || dst_w != frame->width || dst_h != frame->height) {
        dst_w = frame->width;
        dst_h = frame->height;
        state->sws_ctx.reset(sws_getContext(
            frame->width, frame->height, (AVPixelFormat)frame->format, dst_w,
            dst_h, AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr));

        int numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, dst_w, dst_h,
                                                32); // 32位对齐
        local_rgb_buffer.resize(numBytes);
        av_image_fill_arrays(frame_rgb->data, frame_rgb->linesize,
                             local_rgb_buffer.data(), AV_PIX_FMT_RGB24, dst_w,
                             dst_h, 32);
      }

      // ... (执行 sws_scale 和 memcpy 到 video_buf，代码同上个版本) ...
      if (state->sws_ctx) {
        sws_scale(state->sws_ctx.get(), (const uint8_t *const *)frame->data,
                  frame->linesize, 0, frame->height, frame_rgb->data,
                  frame_rgb->linesize);
        {
          std::lock_guard<std::mutex> lock(state->video_mutex);
          if (state->video_buf.size() != local_rgb_buffer.size())
            state->video_buf.resize(local_rgb_buffer.size());
          memcpy(state->video_buf.data(), local_rgb_buffer.data(),
                 local_rgb_buffer.size());

          state->video_buf_w = dst_w;
          state->video_buf_h = dst_h;
          state->video_linesize = frame_rgb->linesize[0];
          state->video_frame_ready = true;
        }
        SDL_Event event;
        event.type = FF_REFRESH_EVENT;
        SDL_PushEvent(&event);
      }
    }
  }
  std::cout << "[VideoDisplayThread] 退出...\n";
}

// ============================================================================
// 4. 音频解码线程 (Audio Decode Thread) - [已修复缓冲区写入逻辑]
// ============================================================================
// --- 4. 音频解码线程 (修复噪音版) ---
void audio_decode_thread_func(std::shared_ptr<PlayerState> state) {
  std::cout << "[AudioDecodeThread] 启动...\n";
  auto frame = make_unique_frame();
  auto frame_converted = make_unique_frame();
  UniqueAVPacket packet;

  while (!state->quit) {
    if (!state->audio_packet_queue.pop(packet))
      break;
    // =================================================
    // 【新增】 响应 Flush 包
    // =================================================
    if (packet->stream_index == -1) {
      // 1. 冲刷解码器
      avcodec_flush_buffers(state->audio_codec_ctx.get());

      // 2. 清空音频环形缓冲区 (防止播放出旧的声音残余)
      {
        std::lock_guard<std::mutex> lock(state->audio_mutex);
        state->audio_ring_buffer.assign(state->audio_ring_buffer.size(),
                                        0); // 清零数据
        state->audio_buf_size = 0;
        state->audio_buf_read_pos = 0;
        state->audio_buf_write_pos = 0;

        // 3. 重置 PI 控制器积分项 (关键！防止旧的累积误差影响 Seek 后的同步)
        state->audio_diff_cum = 0.0;
      }

      std::cout << "[Audio] Flushed & Reset!\n";
      continue;
    }
    // =================================================

    int ret = avcodec_send_packet(state->audio_codec_ctx.get(), packet.get());
    if (ret < 0)
      continue;

    while (ret >= 0) {
      ret = avcodec_receive_frame(state->audio_codec_ctx.get(), frame.get());
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        break;
      else if (ret < 0)
        break;

      // 1. SWR 初始化 (保持不变)
      AVChannelLayout in_layout = frame->ch_layout;
      if (in_layout.nb_channels == 0)
        av_channel_layout_default(&in_layout, 2);

      if (!state->swr_ctx ||
          av_channel_layout_compare(&state->swr_in_ch_layout, &in_layout) !=
              0 ||
          state->swr_in_rate != frame->sample_rate ||
          state->swr_in_fmt != frame->format) {
        state->swr_ctx.reset();
        SwrContext *swr = nullptr;
        swr_alloc_set_opts2(&swr, &state->audio_tgt.channel_layout_struct,
                            state->audio_tgt.fmt, state->audio_tgt.freq,
                            &in_layout, (AVSampleFormat)frame->format,
                            frame->sample_rate, 0, nullptr);
        if (!swr || swr_init(swr) < 0) {
          if (swr)
            swr_free(&swr);
          continue;
        }
        state->swr_ctx.reset(swr);
        av_channel_layout_uninit(&state->swr_in_ch_layout);
        av_channel_layout_copy(&state->swr_in_ch_layout, &in_layout);
        state->swr_in_rate = frame->sample_rate;
        state->swr_in_fmt = (AVSampleFormat)frame->format;
      }

      // 2. 重采样
      if (state->swr_ctx) {
        int out_samples = av_rescale_rnd(
            swr_get_delay(state->swr_ctx.get(), frame->sample_rate) +
                frame->nb_samples,
            state->audio_tgt.freq, frame->sample_rate, AV_ROUND_UP);

        if (frame_converted->nb_samples != out_samples) {
          av_frame_unref(frame_converted.get());
          av_channel_layout_copy(&frame_converted->ch_layout,
                                 &state->audio_tgt.channel_layout_struct);
          frame_converted->format = state->audio_tgt.fmt;
          frame_converted->sample_rate = state->audio_tgt.freq;
          frame_converted->nb_samples = out_samples;
          av_frame_get_buffer(frame_converted.get(), 0);
        }

        int len = swr_convert(state->swr_ctx.get(), frame_converted->data,
                              out_samples, (const uint8_t **)frame->data,
                              frame->nb_samples);

        // 计算字节数：样本数 * 通道数 * 每个样本字节数
        int bytes_per_sample =
            av_get_bytes_per_sample(state->audio_tgt.fmt); // S16 = 2
        int data_size = len * state->audio_tgt.channels * bytes_per_sample;

        // 【关键修复】：确保写入缓冲区时没有处于“半个样本”的状态
        // 虽然 data_size 理论上是对齐的，但这是一个保险

        // =======================================================
        // 【新增】 音频导出分流
        // 这里的 frame 是解码后原始的 frame，或者你可以选择分发 swr 后的数据
        // 建议分发原始 frame，让 Transcoder 内部自己处理重采样，解耦更彻底
        // =======================================================
        if (state->is_exporting && state->transcoder) {
          state->transcoder->PushAudio(frame.get());
        }
        // =======================================================

        // 3. 写入环形缓冲区
        while (!state->quit) {
          std::unique_lock<std::mutex> lock(state->audio_mutex);
          size_t cap = state->audio_ring_buffer.size();
          size_t space_free = cap - state->audio_buf_size;

          if (space_free < data_size) {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
          }

          size_t wpos = state->audio_buf_write_pos;

          // 分两次写入处理回环
          size_t len1 = std::min((size_t)data_size, cap - wpos);
          size_t len2 = data_size - len1;

          memcpy(state->audio_ring_buffer.data() + wpos,
                 frame_converted->data[0], len1);
          if (len2 > 0) {
            memcpy(state->audio_ring_buffer.data(),
                   frame_converted->data[0] + len1, len2);
          }

          state->audio_buf_write_pos = (wpos + data_size) % cap;
          state->audio_buf_size += data_size;

          // 更新时钟：这里使用这一帧的 PTS
          // 只有成功写入了，才更新时钟，这样 current_time 计算才准
          if (frame->pts != AV_NOPTS_VALUE) {
            state->audio_clock =
                frame->pts * av_q2d(state->audio_stream->time_base) +
                (double)frame->nb_samples / frame->sample_rate;
          }
          break;
        }
      }
    }
  }
  std::cout << "[AudioDecodeThread] 退出...\n";
}

// --- 5. SDL 音频回调 ---
void sdl_audio_callback(void *userdata, Uint8 *stream, int len) {
  PlayerState *state = (PlayerState *)userdata;

  // 默认填充静音（非常重要，否则扬声器会播放噪音）
  memset(stream, 0, len);

  if (state->quit)
    return;

  std::lock_guard<std::mutex> lock(state->audio_mutex);

  // 【诊断 3】检查缓冲区状态
  // 为了防止日志刷屏，我们每调用 50 次打印一次 (约 1秒一次)
  static int log_counter = 0;
  if (log_counter++ % 50 == 0) {
    // 打印当前缓冲区有多少数据
    if (state->audio_buf_size == 0) {
      std::cout << "[Audio Callback] Buffer Empty! (Silence)" << std::endl;
    } else {
      std::cout << "[Audio Callback] Playing... BufSize: "
                << state->audio_buf_size << std::endl;
    }
  }

  if (state->audio_buf_size == 0) {
    return; // 缓冲区空，直接返回静音
  }

  int to_read = std::min((size_t)len, state->audio_buf_size);
  size_t rpos = state->audio_buf_read_pos;
  size_t cap = state->audio_ring_buffer.size();

  size_t len1 = std::min((size_t)to_read, cap - rpos);
  memcpy(stream, state->audio_ring_buffer.data() + rpos, len1);
  if (to_read > len1) {
    memcpy(stream + len1, state->audio_ring_buffer.data(), to_read - len1);
  }

  state->audio_buf_read_pos = (rpos + to_read) % cap;
  state->audio_buf_size -= to_read;
}

// 主函数 (修复开头静音版)
// ============================================================================
int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <input_file>\n";
    return 1;
  }

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER) != 0)
    return 1;

  AVFormatContext *fmt_ctx_raw = nullptr;
  if (avformat_open_input(&fmt_ctx_raw, argv[1], nullptr, nullptr) < 0)
    return 1;
  if (avformat_find_stream_info(fmt_ctx_raw, nullptr) < 0)
    return 1;

  auto state = std::make_shared<PlayerState>(fmt_ctx_raw);
  state->video_stream_index =
      find_stream_index(state->ic.get(), AVMEDIA_TYPE_VIDEO);
  state->audio_stream_index =
      find_stream_index(state->ic.get(), AVMEDIA_TYPE_AUDIO);

  // Init Video
  if (state->video_stream_index >= 0) {
    state->video_stream = state->ic->streams[state->video_stream_index];
    const AVCodec *codec =
        avcodec_find_decoder(state->video_stream->codecpar->codec_id);
    AVCodecContext *ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(ctx, state->video_stream->codecpar);
    ctx->thread_count = 0;
    if (codec->capabilities & AV_CODEC_CAP_FRAME_THREADS)
      ctx->thread_type = FF_THREAD_FRAME;
    else if (codec->capabilities & AV_CODEC_CAP_SLICE_THREADS)
      ctx->thread_type = FF_THREAD_SLICE;
    avcodec_open2(ctx, codec, nullptr);
    state->video_codec_ctx.reset(ctx);
  }

  // Init Audio
  SDL_AudioDeviceID audio_dev_id = 0;
  if (state->audio_stream_index >= 0) {
    state->audio_stream = state->ic->streams[state->audio_stream_index];
    const AVCodec *codec =
        avcodec_find_decoder(state->audio_stream->codecpar->codec_id);
    AVCodecContext *ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(ctx, state->audio_stream->codecpar);
    avcodec_open2(ctx, codec, nullptr);
    state->audio_codec_ctx.reset(ctx);

    SDL_AudioSpec wanted{}, spec{};
    wanted.freq = 44100;
    wanted.format = AUDIO_S16SYS;
    wanted.channels = 2;
    wanted.samples = 1024;
    wanted.callback = sdl_audio_callback;
    wanted.userdata = state.get();

    audio_dev_id = SDL_OpenAudioDevice(nullptr, 0, &wanted, &spec, 0);

    // 【关键修改 1】这里参数填 1 (PAUSED)，先不要播放！等数据够了再播
    SDL_PauseAudioDevice(audio_dev_id, 1);

    state->audio_tgt.freq = spec.freq;
    state->audio_tgt.fmt = AV_SAMPLE_FMT_S16;
    state->audio_tgt.channels = spec.channels;
    state->audio_tgt.bytes_per_sec = spec.freq * spec.channels * 2;
    av_channel_layout_default(&state->audio_tgt.channel_layout_struct,
                              spec.channels);
  }

  // Window
  int w = 800, h = 600;
  if (state->video_codec_ctx) {
    w = state->video_codec_ctx->width;
    h = state->video_codec_ctx->height;
    if (w > 1280) {
      h = h * 1280 / w;
      w = 1280;
    }
  }
  SDL_Window *window = SDL_CreateWindow(
      "FFmpeg Player", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w, h,
      SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
  SDL_Renderer *renderer =
      SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
  SDL_Texture *texture = nullptr;

  // Start Threads
  state->status = PlayerStatus::PLAYING;
  std::vector<std::thread> threads;
  threads.emplace_back(read_thread_func, state);
  if (state->video_stream_index >= 0) {
    threads.emplace_back(video_decode_thread_func, state);
    threads.emplace_back(video_display_thread_func, state);
  }
  if (state->audio_stream_index >= 0) {
    threads.emplace_back(audio_decode_thread_func, state);
  }

  // Event Loop
  bool audio_started = false; // 标记音频是否已启动

  SDL_Event event;
  while (!state->quit) {
    // 【关键修改 2】这里不能用
    // WaitEvent，因为它会无限阻塞，导致无法检查音频缓冲 改用 PollEvent +
    // sleep，或者 WaitEventTimeout
    if (SDL_WaitEventTimeout(&event, 10)) { // 10ms 超时
      if (event.type == SDL_QUIT) {
        state->quit = true;
      } else if (event.type == FF_REFRESH_EVENT) {
        std::lock_guard<std::mutex> lock(state->video_mutex);
        if (state->video_frame_ready && !state->video_buf.empty()) {
          if (!texture || state->video_buf_w != w || state->video_buf_h != h) {
            if (texture)
              SDL_DestroyTexture(texture);
            w = state->video_buf_w;
            h = state->video_buf_h;
            // SDL_SetWindowSize(window, w, h);
            texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24,
                                        SDL_TEXTUREACCESS_STREAMING, w, h);
          }

          if (texture) {
            SDL_UpdateTexture(texture, nullptr, state->video_buf.data(),
                              state->video_linesize);
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, nullptr, nullptr);
            if (state->mv_visualization_enabled) {
              SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
              for (const auto &mv : state->mvs) {
                if (mv.motion_x == 0 && mv.motion_y == 0)
                  continue;
                SDL_RenderDrawLine(renderer, mv.dst_x, mv.dst_y, mv.src_x,
                                   mv.src_y);
              }
            }
            SDL_RenderPresent(renderer);
          }
          state->video_frame_ready = false;
        }
      } else if (event.type == SDL_KEYDOWN) {
        // 【核心修复】如果这是由长按产生的重复事件，直接跳过，不处理！
        if (event.key.repeat != 0) {
          continue;
        }

        if (event.key.keysym.sym == SDLK_SPACE) {
          if (state->status == PlayerStatus::PLAYING) {
            state->status = PlayerStatus::PAUSED;
            if (audio_dev_id)
              SDL_PauseAudioDevice(audio_dev_id, 1);
          } else {
            state->status = PlayerStatus::PLAYING;
            if (audio_dev_id)
              SDL_PauseAudioDevice(audio_dev_id, 0);
          }
        }

        // =======================================================
        // 【新增】 按 'R' 键开始/停止录制
        // =======================================================
        else if (event.key.keysym.sym == SDLK_r) {
          if (!state->is_exporting) {
            std::cout << "[Main] === 开始录制 (CRF 模式) ===\n";
            state->transcoder = std::make_shared<VideoTranscoder>();

            // 1. 获取当前视频参数
            int w = state->video_codec_ctx->width;
            int h = state->video_codec_ctx->height;

            // 2. 获取帧率 (如果获取不到则默认 25)
            int fps = 25;
            if (state->video_stream->avg_frame_rate.den > 0) {
              fps = state->video_stream->avg_frame_rate.num /
                    state->video_stream->avg_frame_rate.den;
            }
            // 2. 【新增】 获取音频参数
            // 从音频解码器上下文获取源参数
            int a_channels = state->audio_codec_ctx->ch_layout.nb_channels;
            int a_rate = state->audio_codec_ctx->sample_rate;
            AVChannelLayout a_layout = state->audio_codec_ctx->ch_layout;

            // 3. 初始化 (参数变多了)
            if (state->transcoder->Init("../tests_vedios/output.mp4", w, h, fps,
                                        a_channels, a_rate,
                                        a_layout, // 传入音频参数
                                        RateControlMode::CRF)) {
              state->transcoder->Start();
              state->is_exporting = true;
            } else {
              std::cerr << "[Main] 录制初始化失败!\n";
            }
          } else {
            std::cout << "[Main] === 停止录制 ===\n";
            state->is_exporting = false;
            // 4. 异步销毁转码器 (写文件尾比较耗时，别卡住 UI)
            std::thread([t = state->transcoder]() {
              t->Stop(); // 内部会等待编码线程结束、Flush编码器、写文件尾
              std::cout << "[Main] 录制文件已保存至 output.mp4\n";
            }).detach();
            state->transcoder = nullptr;
          }
        }
        // =================================================
        // 【新增】 左右方向键 Seek
        // =================================================
        else if (event.key.keysym.sym == SDLK_RIGHT) {
          // 快进 10 秒
          double incr = 1.0;
          double current_pos = state->audio_clock; // 获取当前播放时间
          double target_pos = current_pos + incr;

          // 计算目标时间戳 (微秒)
          int64_t target_ts = (int64_t)(target_pos * AV_TIME_BASE);

          // 线程安全地提交请求
          {
            std::lock_guard<std::mutex> lock(state->seek_mutex);
            state->seek_pos = target_ts;
            state->seek_request = true;
          }
          std::cout << "[Main] Seek Forward to: " << target_pos << "s\n";
        } else if (event.key.keysym.sym == SDLK_LEFT) {
          // 快退 10 秒
          double incr = -10.0;
          double current_pos = state->audio_clock;
          double target_pos = current_pos + incr;
          if (target_pos < 0)
            target_pos = 0; // 防止负数

          int64_t target_ts = (int64_t)(target_pos * AV_TIME_BASE);

          {
            std::lock_guard<std::mutex> lock(state->seek_mutex);
            state->seek_pos = target_ts;
            state->seek_request = true;
          }
          std::cout << "[Main] Seek Backward to: " << target_pos << "s\n";
        }
        // =================================================
      }
      // =======================================================
    }

    // 【关键修改 3】检查是否应该开启音频
    // 如果还没开启，且缓冲区里有足够的数据（比如 > 0 字节 或者 更多，这里用 >0
    // 即可，因为解码器很快） 就开启音频设备
    if (!audio_started && audio_dev_id != 0) {
      size_t buf_size = 0;
      {
        std::lock_guard<std::mutex> lock(state->audio_mutex);
        buf_size = state->audio_buf_size;
      }
      // 只要有数据了，就开始播放。
      // 更稳健的做法是等待 buf_size > 最小缓冲区大小(比如1024*4)
      if (buf_size > 0) {
        std::cout << "[Main] Audio buffer filled (" << buf_size
                  << "), starting playback...\n";
        SDL_PauseAudioDevice(audio_dev_id, 0); // 0 = Play
        audio_started = true;
      }
    }
  }

  // Cleanup
  state->video_packet_queue.abort();
  state->video_frame_queue.abort();
  state->audio_packet_queue.abort();
  state->audio_frame_queue.abort();

  for (auto &t : threads)
    if (t.joinable())
      t.join();

  if (texture)
    SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}