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
#include "../includes/YUVRenderer.h"
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
// --- 1. 读取线程 (支持 Seek 与 软硬双模) ---
// 注意：函数签名增加了 url 参数，用于判断流类型
void read_thread_func(std::shared_ptr<PlayerState> state, std::string url) {
  std::cout << "[ReadThread] 启动...\n";

  // 辅助 lambda：创建一个特殊的 Flush 包 (stream_index = -1)
  // 解码线程收到这个包后，会清空解码器缓存，消除花屏
  auto make_flush_packet = []() {
    auto pkt = make_unique_packet();
    pkt->stream_index = -1; // 约定 -1 为 Flush 信号
    return pkt;
  };

  auto packet_raw = make_unique_packet();
  int error_count = 0;

  // 判断是否为直播流 (简单判断：URL 以 rtsp:// 开头)
  // 直播流严禁 Seek，否则会导致连接断开
  bool is_live_stream = (url.find("rtsp://") == 0);

  while (!state->quit) {

    // =================================================
    // 【核心逻辑】 处理 Seek 请求
    // =================================================
    if (state->seek_request) {
      // 场景 A：直播流 -> 禁止 Seek
      if (is_live_stream) {
        std::cout << "[ReadThread] 警告：直播流不支持 Seek 操作！\n";
        state->seek_request = false; // 消费掉请求，不做任何事
      }
      // 场景 B：本地文件 -> 执行 Seek
      else {
        int64_t seek_target = state->seek_pos;
        int64_t seek_min = INT64_MIN;
        int64_t seek_max = INT64_MAX;

        std::cout << "[ReadThread] 执行 Seek 跳转到: " << seek_target
                  << " (微秒)...\n";

        // 1. 调用 FFmpeg API (AVSEEK_FLAG_BACKWARD 找最近的关键帧)
        int ret = avformat_seek_file(state->ic.get(), -1, seek_min, seek_target,
                                     seek_max, AVSEEK_FLAG_BACKWARD);

        if (ret >= 0) {
          // 2. Seek 成功后，必须“冲刷”旧数据

          // 处理视频队列
          if (state->video_stream_index >= 0) {
            state->video_packet_queue.flush(); // 清空队列中积压的旧包
            state->video_packet_queue.push(
                make_flush_packet()); // 发送红牌，通知解码器 Flush
          }

          // 处理音频队列
          if (state->audio_stream_index >= 0) {
            state->audio_packet_queue.flush();                   // 清空队列
            state->audio_packet_queue.push(make_flush_packet()); // 发送红牌
          }

          // 3. (可选) 如果之前是 EOF 状态，现在 Seek 回去了，要重置 EOF 标志
          // 在这里简单通过 continue 恢复读取循环即可
        } else {
          std::cerr << "[ReadThread] Seek 失败!\n";
        }

        // 消费请求
        state->seek_request = false;

        // Seek 后为了防止读取逻辑混乱，直接跳过本次循环，重新开始读取
        continue;
      }
    }
    // =================================================

    av_packet_unref(packet_raw.get());

    // 读取一帧 (可能会阻塞)
    int ret = av_read_frame(state->ic.get(), packet_raw.get());

    if (ret < 0) {
      if (ret == AVERROR_EOF) {
        // 文件读完了
        // 只有在没有 Seek 请求的时候才发送 EOF 包
        // 防止 Seek 到末尾附近时误判
        if (!state->seek_request) {
          if (state->video_stream_index >= 0)
            state->video_packet_queue.push(make_unique_packet()); // 空包 = EOF
          if (state->audio_stream_index >= 0)
            state->audio_packet_queue.push(make_unique_packet());

          std::cout << "[ReadThread] 已播放至文件末尾 (EOF).\n";

          // 对于本地文件，读完后休眠等待用户操作 (Seek 或 退出)
          // 不要直接 break 退出线程，否则就没法 Seek 回去重播了
          while (!state->quit && !state->seek_request) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
          continue;
        }
      } else {
        // 网络/文件读取错误处理
        char errbuf[1024];
        av_strerror(ret, errbuf, 1024);
        // 过滤掉这就退出的错误，防止日志刷屏
        if (error_count % 50 == 0) {
          std::cerr << "[ReadThread] 读取错误: " << errbuf << "\n";
        }

        error_count++;
        if (error_count > 500) { // 容忍度调高一点
          std::cerr << "[ReadThread] 连续错误过多，断开连接！\n";
          state->quit = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
    }
    error_count = 0;

    // 分发 Packet 到对应的队列
    if (packet_raw->stream_index == state->video_stream_index) {
      auto pkt = make_unique_packet();
      av_packet_move_ref(pkt.get(), packet_raw.get()); // 零拷贝移动数据
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

// --- 3. 视频显示线程 (OpenGL 版) ---
void video_display_thread_func(std::shared_ptr<PlayerState> state) {
  std::cout << "[VideoDisplayThread] 启动 (OpenGL Mode)...\n";

  UniqueAVFrame frame;

  // 【1. 创建渲染器实例】
  // OpenGL 上下文通常需要在渲染线程初始化，或者在主线程初始化后 MakeCurrent
  // 简单的做法：在主线程创建 Window，在这里创建 OpenGL Context (通过
  // YUVRenderer::Init)
  YUVRenderer renderer;
  bool renderer_inited = false;

  // --- 同步阈值常量 (保持不变) ---
  const double AV_SYNC_THRESHOLD_MIN = 0.04;
  const double AV_SYNC_THRESHOLD_MAX = 0.1;
  const double KP = 0.8;
  const double KI = 0.005;
  const double MAX_INTEGRAL = 1.0;

  // 在 while 循环外定义起播计数器
  int startup_frame_count = 0;
  const int STARTUP_IGNORE_SYNC_FRAMES = 60; // 前60帧起播缓冲

  while (!state->quit) {
    frame.reset();
    if (state->status == PlayerStatus::PAUSED) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    if (!state->video_frame_queue.pop(frame))
      break;
    if (!frame)
      break;

    // --- 同步逻辑开始 (完全保留你之前的优秀逻辑) ---
    if (frame->pts != AV_NOPTS_VALUE) {
      double video_clock = frame->pts * av_q2d(state->video_stream->time_base);
      double frame_duration = 0.0;
      if (state->frame_last_pts != 0.0) {
        frame_duration = video_clock - state->frame_last_pts;
      }
      if (frame_duration <= 0 || frame_duration > 10.0) {
        frame_duration = state->frame_last_delay;
      }

      state->frame_last_pts = video_clock;
      state->frame_last_delay = frame_duration;

      bool is_startup = (startup_frame_count < STARTUP_IGNORE_SYNC_FRAMES);
      startup_frame_count++;

      double delay = frame_duration;

      if (is_startup) {
        // === 状态 A：起播态 ===
        // 策略：完整性优先。
        // 哪怕视频时间戳落后于音频，也不丢帧，而是按正常速度（frame_duration）播放。
        // 这让用户能看到流畅的“缓冲画面”，而不是黑屏。
        delay = frame_duration; // 起播阶段正常速度播放，保证画面完整

        // 特殊处理：绝对的第一帧
        // 起播时，第一帧必须立即渲染，不能等待音频同步。
        // 这是因为第一帧的显示时间点是未知的，必须立即渲染。
        if (startup_frame_count == 1) {
          delay = 0.0;
          state->frame_timer = (double)av_gettime_relative() / 1000000.0;
          std::cout << "[VideoSync] 起播开始！OpenGL 准备渲染...\n";
        }
      } else {
        // === 状态 B：稳态 ===
        // 策略：实时性优先。
        // 启用 PI 控制算法进行微调。
        // 启用丢帧策略：如果 diff < -0.08s (严重滞后)，则丢包追赶。
        double audio_clock = state->audio_clock;
        double buffered_duration = 0.0;
        {
          std::lock_guard<std::mutex> lock(state->audio_mutex);
          if (state->audio_tgt.bytes_per_sec > 0) {
            buffered_duration =
                (double)state->audio_buf_size / state->audio_tgt.bytes_per_sec;
          }
        }
        double current_audio_time = audio_clock - buffered_duration;
        double diff = video_clock - current_audio_time;

        double p_adjust = KP * diff;
        state->audio_diff_cum += diff * frame_duration;
        state->audio_diff_cum = std::min(
            MAX_INTEGRAL, std::max(-MAX_INTEGRAL, state->audio_diff_cum));
        double i_adjust = KI * state->audio_diff_cum;

        delay += (p_adjust + i_adjust);

        if (diff < -2.0 * frame_duration) {
          av_frame_unref(frame.get());
          continue;
        }
        delay = std::min(delay, 2.0 * frame_duration);
        delay = std::max(delay, frame_duration * 0.5);
      }

      double time = (double)av_gettime_relative() / 1000000.0;
      double remaining_time = state->frame_timer + delay - time;

      if (remaining_time > 0) {
        av_usleep((unsigned)(remaining_time * 1000000.0));
      }

      if (remaining_time <= 0 && is_startup) {
        state->frame_timer = time;
      } else {
        state->frame_timer += delay;
      }
    }
    // --- 同步逻辑结束 ---

    // =================================================
    // 【OpenGL 渲染核心修改】
    // =================================================
    if (frame->width > 0 && frame->height > 0) {

      // 1. 延迟初始化渲染器 (需要拿到宽高)
      if (!renderer_inited) {
        // 这里的 state->window 是在 main 中创建的 SDL_Window
        if (renderer.Init(state->window, frame->width, frame->height)) {
          renderer_inited = true;
          std::cout << "[OpenGL] Init Success: " << frame->width << "x"
                    << frame->height << std::endl;

          // 顺便把窗口大小调整好
          SDL_SetWindowSize(state->window, frame->width, frame->height);
          SDL_SetWindowPosition(state->window, SDL_WINDOWPOS_CENTERED,
                                SDL_WINDOWPOS_CENTERED);
        } else {
          std::cerr << "[OpenGL] Init Failed! Quit.\n";
          state->quit = true;
          break;
        }
      }

      // 2. 直接渲染 YUV 数据 (零拷贝核心)
      // 注意：这里不再需要 sws_scale，直接把 data[0], data[1], data[2] 喂给 GPU
      renderer.Render(frame->data[0], frame->data[1], frame->data[2],
                      frame->linesize[0], frame->linesize[1],
                      frame->linesize[2]);

      // 注意：Render 内部调用了
      // SDL_GL_SwapWindow，所以这里不需要发事件给主线程刷新 除非你的 UI
      // 逻辑必须在主线程跑
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

// 主函数 (OpenGL + Seek + Record 最终版)
// ============================================================================
int main(int argc, char **argv) {
  std::string input_url = "rtsp://127.0.0.1/live/test";
  if (argc > 1) {
    input_url = argv[1];
  }

  // SDL 初始化
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER) != 0)
    return 1;

  // 网络流配置
  AVFormatContext *fmt_ctx_raw = avformat_alloc_context();
  AVDictionary *options = nullptr;

  // 针对 RTSP 的低延迟优化
  // 添加 TCP 模式，防止 UDP 丢包
  // 同时设置较大的 buffer_size 和 probesize，提高首屏加载速度
  if (input_url.find("rtsp://") == 0) {
    av_dict_set(&options, "rtsp_transport", "tcp", 0);
    av_dict_set(&options, "buffer_size", "102400", 0);
    av_dict_set(&options, "probesize", "102400", 0);
    av_dict_set(&options, "analyzeduration", "100000", 0);
    av_dict_set(&options, "stimeout", "2000000", 0);
  }

  std::cout << "[Main] 连接中: " << input_url << " ...\n";

  if (avformat_open_input(&fmt_ctx_raw, input_url.c_str(), nullptr, &options) <
      0)
    return 1;
  if (avformat_find_stream_info(fmt_ctx_raw, nullptr) < 0)
    return 1;

  std::cout << "[Main] 连接成功！\n";

  auto state = std::make_shared<PlayerState>(fmt_ctx_raw);
  state->video_stream_index =
      find_stream_index(state->ic.get(), AVMEDIA_TYPE_VIDEO);
  state->audio_stream_index =
      find_stream_index(state->ic.get(), AVMEDIA_TYPE_AUDIO);

  // Init Video Decoder
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

  // Init Audio Decoder & SDL Audio
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
    SDL_PauseAudioDevice(audio_dev_id, 1);

    state->audio_tgt.freq = spec.freq;
    state->audio_tgt.fmt = AV_SAMPLE_FMT_S16;
    state->audio_tgt.channels = spec.channels;
    state->audio_tgt.bytes_per_sec = spec.freq * spec.channels * 2;
    av_channel_layout_default(&state->audio_tgt.channel_layout_struct,
                              spec.channels);
  }

  // =================================================
  // 【OpenGL 修改 1】创建窗口 (移除 Renderer)
  // =================================================
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
      "FFmpeg Player (OpenGL)", SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED, w, h, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

  state->window = window; // 存入 state，供渲染线程使用

  // Start Threads
  state->status = PlayerStatus::PLAYING;
  std::vector<std::thread> threads;

  // 【核心修改 2】传入 URL 给 read_thread_func
  threads.emplace_back(read_thread_func, state, input_url);

  if (state->video_stream_index >= 0) {
    threads.emplace_back(video_decode_thread_func, state);
    threads.emplace_back(video_display_thread_func, state); // 负责 OpenGL 渲染
  }
  if (state->audio_stream_index >= 0) {
    threads.emplace_back(audio_decode_thread_func, state);
  }

  // Event Loop
  bool audio_started = false;
  SDL_Event event;

  while (!state->quit) {
    if (SDL_WaitEventTimeout(&event, 10)) {
      if (event.type == SDL_QUIT) {
        state->quit = true;
      }
      // 【OpenGL 修改 3】删除了 FF_REFRESH_EVENT 处理

      else if (event.type == SDL_KEYDOWN) {
        if (event.key.repeat != 0)
          continue;

        // --- 1. 暂停/播放 ---
        if (event.key.keysym.sym == SDLK_SPACE) {
          if (state->status == PlayerStatus::PLAYING) {
            state->status = PlayerStatus::PAUSED;
            if (audio_dev_id)
              SDL_PauseAudioDevice(audio_dev_id, 1);
            std::cout << "[Control] Pause\n";
          } else {
            state->status = PlayerStatus::PLAYING;
            if (audio_dev_id)
              SDL_PauseAudioDevice(audio_dev_id, 0);
            std::cout << "[Control] Play\n";
          }
        }
        // --- 2. 录制控制 (R键) ---
        else if (event.key.keysym.sym == SDLK_r) {
          if (!state->is_exporting) {
            std::cout << "[Control] === 开始录制 (CRF 模式) ===\n";
            state->transcoder = std::make_shared<VideoTranscoder>();
            int tw = state->video_codec_ctx->width;
            int th = state->video_codec_ctx->height;
            int fps = 25;
            if (state->video_stream->avg_frame_rate.den > 0)
              fps = state->video_stream->avg_frame_rate.num /
                    state->video_stream->avg_frame_rate.den;
            int a_ch = state->audio_codec_ctx->ch_layout.nb_channels;
            int a_r = state->audio_codec_ctx->sample_rate;
            AVChannelLayout a_lay = state->audio_codec_ctx->ch_layout;

            if (state->transcoder->Init("output.mp4", tw, th, fps, a_ch, a_r,
                                        a_lay, RateControlMode::CRF)) {
              state->transcoder->Start();
              state->is_exporting = true;
            }
          } else {
            std::cout << "[Control] === 停止录制 ===\n";
            state->is_exporting = false;
            std::thread([t = state->transcoder]() { t->Stop(); }).detach();
            state->transcoder = nullptr;
          }
        }
        // --- 3. Seek 控制 (左右方向键) ---
        else if (event.key.keysym.sym == SDLK_RIGHT ||
                 event.key.keysym.sym == SDLK_LEFT) {
          double incr = (event.key.keysym.sym == SDLK_RIGHT) ? 10.0 : -10.0;
          double current_pos = state->audio_clock;
          double target_pos = current_pos + incr;
          if (target_pos < 0)
            target_pos = 0;

          int64_t target_ts = (int64_t)(target_pos * AV_TIME_BASE);

          {
            std::lock_guard<std::mutex> lock(state->seek_mutex);
            state->seek_pos = target_ts;
            state->seek_request = true;
          }
          std::cout << "[Control] Request Seek: " << (incr > 0 ? "+" : "")
                    << incr << "s\n";
        }
      }
    }

    // 音频起播检查 (保持不变)
    if (!audio_started && audio_dev_id != 0) {
      size_t buf_size = 0;
      {
        std::lock_guard<std::mutex> lock(state->audio_mutex);
        buf_size = state->audio_buf_size;
      }
      const size_t MIN_PRE_ROLL = 1024 * 2 * 2 * 3;
      if (buf_size > MIN_PRE_ROLL) {
        std::cout << "[Main] Audio buffer filled (" << buf_size
                  << "), starting playback...\n";
        SDL_PauseAudioDevice(audio_dev_id, 0);
        audio_started = true;
      }
    }
  }

  // Cleanup (保持不变)
  state->video_packet_queue.abort();
  state->video_frame_queue.abort();
  state->audio_packet_queue.abort();
  state->audio_frame_queue.abort();

  for (auto &t : threads)
    if (t.joinable())
      t.join();

  // 【OpenGL 修改 4】不再销毁 Texture/Renderer
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}