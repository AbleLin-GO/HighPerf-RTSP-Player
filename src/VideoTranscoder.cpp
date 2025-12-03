#include "../includes/VideoTranscoder.h"
#include <iostream>

extern "C" {
#include <libavutil/channel_layout.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/time.h>
}

VideoTranscoder::VideoTranscoder() {}

VideoTranscoder::~VideoTranscoder() {
  Stop();
  // 释放手动管理的 C 资源
  if (audio_fifo_) {
    av_audio_fifo_free(audio_fifo_);
    audio_fifo_ = nullptr;
  }
  // 释放 layout
  av_channel_layout_uninit(&src_ch_layout_);
}

// --- 1. 补回丢失的配置函数 ---
void VideoTranscoder::ConfigureRateControl(AVCodecContext *ctx,
                                           RateControlMode mode, int bitrate) {
  // 基础码率设置 (bps)
  ctx->bit_rate = bitrate * 1000;

  // x264 预设：平衡速度和压缩率
  av_opt_set(ctx->priv_data, "preset", "veryfast", 0);

  switch (mode) {
  case RateControlMode::CRF:
    // CRF 模式：画质优先，适合本地导出
    av_opt_set(ctx->priv_data, "crf", "28", 0);
    ctx->rc_max_rate = 0;
    break;
  case RateControlMode::CBR_VBV:
    // CBR 模式：模拟推流，限制最大码率和缓冲区
    ctx->rc_min_rate = ctx->bit_rate;
    ctx->rc_max_rate = ctx->bit_rate;
    ctx->rc_buffer_size = ctx->bit_rate * 2;
    break;
  case RateControlMode::ABR:
    // ABR 模式
    ctx->rc_max_rate = ctx->bit_rate * 1.5;
    ctx->rc_buffer_size = ctx->bit_rate * 2;
    break;
  }
}

// --- 2. 包含音频参数的 Init ---
bool VideoTranscoder::Init(const std::string &filename, int width, int height,
                           int fps, int audio_channels, int audio_rate,
                           AVChannelLayout audio_layout, RateControlMode mode,
                           int bitrate_kbps) {
  // 1. 封装上下文
  AVFormatContext *temp_fmt = nullptr;
  if (avformat_alloc_output_context2(&temp_fmt, nullptr, nullptr,
                                     filename.c_str()) < 0)
    return false;
  fmt_ctx_.reset(temp_fmt);

  // --- 视频初始化 ---
  const AVCodec *v_codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  if (!v_codec)
    return false;
  out_stream_ = avformat_new_stream(fmt_ctx_.get(), nullptr);
  enc_ctx_.reset(avcodec_alloc_context3(v_codec));

  enc_ctx_->width = width;
  enc_ctx_->height = height;
  enc_ctx_->time_base = {1, fps};
  enc_ctx_->framerate = {fps, 1};
  enc_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
  enc_ctx_->gop_size = fps * 2;

  // 调用配置函数
  ConfigureRateControl(enc_ctx_.get(), mode, bitrate_kbps);

  if (fmt_ctx_->oformat->flags & AVFMT_GLOBALHEADER)
    enc_ctx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  if (avcodec_open2(enc_ctx_.get(), v_codec, nullptr) < 0)
    return false;
  avcodec_parameters_from_context(out_stream_->codecpar, enc_ctx_.get());

  // --- 音频初始化 ---
  src_sample_rate_ = audio_rate;
  av_channel_layout_copy(&src_ch_layout_, &audio_layout);
  src_sample_fmt_ = AV_SAMPLE_FMT_FLTP;

  if (!InitAudio(audio_channels, audio_rate, audio_layout)) {
    std::cerr << "[Transcoder] Audio Init Failed!\n";
    return false;
  }

  // --- 打开 IO 并写入头 ---
  if (!(fmt_ctx_->oformat->flags & AVFMT_NOFILE)) {
    if (avio_open(&fmt_ctx_->pb, filename.c_str(), AVIO_FLAG_WRITE) < 0)
      return false;
  }
  return avformat_write_header(fmt_ctx_.get(), nullptr) >= 0;
}

// --- 3. 音频初始化逻辑 ---
bool VideoTranscoder::InitAudio(int channels, int rate,
                                AVChannelLayout layout) {
  const AVCodec *a_codec = avcodec_find_encoder(AV_CODEC_ID_AAC);
  if (!a_codec)
    return false;

  audio_out_stream_ = avformat_new_stream(fmt_ctx_.get(), nullptr);
  if (!audio_out_stream_)
    return false;

  audio_enc_ctx_.reset(avcodec_alloc_context3(a_codec));
  if (!audio_enc_ctx_)
    return false;

  // AAC 常用参数
  audio_enc_ctx_->sample_fmt =
      a_codec->sample_fmts ? a_codec->sample_fmts[0] : AV_SAMPLE_FMT_FLTP;
  audio_enc_ctx_->bit_rate = 128000; // 128kbps
  audio_enc_ctx_->sample_rate = rate;
  if (av_channel_layout_copy(&audio_enc_ctx_->ch_layout, &layout) < 0)
    return false;

  audio_enc_ctx_->time_base = {1, rate};

  if (fmt_ctx_->oformat->flags & AVFMT_GLOBALHEADER)
    audio_enc_ctx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

  if (avcodec_open2(audio_enc_ctx_.get(), a_codec, nullptr) < 0)
    return false;
  if (avcodec_parameters_from_context(audio_out_stream_->codecpar,
                                      audio_enc_ctx_.get()) < 0)
    return false;

  // 初始化 FIFO
  audio_fifo_ = av_audio_fifo_alloc(audio_enc_ctx_->sample_fmt,
                                    audio_enc_ctx_->ch_layout.nb_channels, 1);
  if (!audio_fifo_)
    return false;

  return true;
}

// --- 4. 补回丢失的 Start 和 Stop ---
void VideoTranscoder::Start() {
  running_ = true;
  finish_signal_ = false;
  encode_thread_ = std::thread(&VideoTranscoder::EncodeThreadFunc, this);
}

void VideoTranscoder::Stop() {
  if (!running_)
    return;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    finish_signal_ = true;
  }
  queue_cv_.notify_all();

  if (encode_thread_.joinable()) {
    encode_thread_.join();
  }

  if (fmt_ctx_) {
    av_write_trailer(fmt_ctx_.get());
    if (!(fmt_ctx_->oformat->flags & AVFMT_NOFILE)) {
      avio_closep(&fmt_ctx_->pb);
    }
  }
  running_ = false;
}

// --- 5. 推流函数 ---
void VideoTranscoder::PushFrame(const AVFrame *src_frame) {
  if (!running_)
    return;
  auto new_frame = make_unique_frame();

  new_frame->format = src_frame->format;
  new_frame->width = src_frame->width;
  new_frame->height = src_frame->height;

  if (av_frame_get_buffer(new_frame.get(), 32) < 0)
    return;
  if (av_frame_copy(new_frame.get(), src_frame) < 0)
    return;

  new_frame->pts = video_pts_ctr_++;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    video_queue_.push(std::move(new_frame));
  }
  queue_cv_.notify_one();
}

void VideoTranscoder::PushAudio(const AVFrame *src_audio) {
  if (!running_)
    return;

  auto new_frame = make_unique_frame();
  new_frame->format = src_audio->format;
  new_frame->sample_rate = src_audio->sample_rate;
  new_frame->nb_samples = src_audio->nb_samples;
  av_channel_layout_copy(&new_frame->ch_layout, &src_audio->ch_layout);

  if (av_frame_get_buffer(new_frame.get(), 0) < 0)
    return;
  if (av_frame_copy(new_frame.get(), src_audio) < 0)
    return;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    audio_queue_.push(std::move(new_frame));
  }
  queue_cv_.notify_one();
}

// --- 6. 核心编码循环 ---
void VideoTranscoder::EncodeThreadFunc() {
  auto pkt = make_unique_packet();

  while (true) {
    UniqueAVFrame v_frame, a_frame;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] {
        return !video_queue_.empty() || !audio_queue_.empty() || finish_signal_;
      });

      if (video_queue_.empty() && audio_queue_.empty() && finish_signal_) {
        // 冲刷
        avcodec_send_frame(enc_ctx_.get(), nullptr);
        FlushAudio();
        avcodec_send_frame(audio_enc_ctx_.get(), nullptr);
        break;
      }

      if (!video_queue_.empty()) {
        v_frame = std::move(video_queue_.front());
        video_queue_.pop();
      }
      if (!audio_queue_.empty()) {
        a_frame = std::move(audio_queue_.front());
        audio_queue_.pop();
      }
    }

    // 视频编码
    if (v_frame) {
      int ret = avcodec_send_frame(enc_ctx_.get(), v_frame.get());
      if (ret >= 0) {
        while (true) {
          ret = avcodec_receive_packet(enc_ctx_.get(), pkt.get());
          if (ret < 0)
            break;
          av_packet_rescale_ts(pkt.get(), enc_ctx_->time_base,
                               out_stream_->time_base);
          pkt->stream_index = out_stream_->index;
          av_interleaved_write_frame(fmt_ctx_.get(), pkt.get());
          av_packet_unref(pkt.get());
        }
      }
    }

    // 音频处理
    if (a_frame) {
      ProcessAudioPacket(a_frame.get());
    }
  }
  std::cout << "[Transcoder] Thread Exit.\n";
}

// --- 7. 音频处理与 Flush ---
void VideoTranscoder::ProcessAudioPacket(AVFrame *frame) {
  if (!swr_ctx_ || frame->format != src_sample_fmt_ ||
      frame->sample_rate != src_sample_rate_) {
    src_sample_fmt_ = (AVSampleFormat)frame->format;
    src_sample_rate_ = frame->sample_rate;

    SwrContext *s = nullptr;
    swr_alloc_set_opts2(&s, &audio_enc_ctx_->ch_layout,
                        audio_enc_ctx_->sample_fmt, audio_enc_ctx_->sample_rate,
                        &frame->ch_layout, (AVSampleFormat)frame->format,
                        frame->sample_rate, 0, nullptr);
    swr_init(s);
    swr_ctx_.reset(s);
  }

  int out_count = av_rescale_rnd(
      swr_get_delay(swr_ctx_.get(), frame->sample_rate) + frame->nb_samples,
      audio_enc_ctx_->sample_rate, frame->sample_rate, AV_ROUND_UP);

  uint8_t **converted_data = nullptr;
  av_samples_alloc_array_and_samples(&converted_data, nullptr,
                                     audio_enc_ctx_->ch_layout.nb_channels,
                                     out_count, audio_enc_ctx_->sample_fmt, 0);

  int converted_samples =
      swr_convert(swr_ctx_.get(), converted_data, out_count,
                  (const uint8_t **)frame->data, frame->nb_samples);

  av_audio_fifo_write(audio_fifo_, (void **)converted_data, converted_samples);

  if (converted_data) {
    av_freep(&converted_data[0]);
    free(converted_data);
  }

  int enc_frame_size = audio_enc_ctx_->frame_size;
  auto temp_frame = make_unique_frame();
  auto pkt = make_unique_packet();

  while (av_audio_fifo_size(audio_fifo_) >= enc_frame_size) {
    temp_frame->nb_samples = enc_frame_size;
    temp_frame->format = audio_enc_ctx_->sample_fmt;
    temp_frame->sample_rate = audio_enc_ctx_->sample_rate;
    av_channel_layout_copy(&temp_frame->ch_layout, &audio_enc_ctx_->ch_layout);
    av_frame_get_buffer(temp_frame.get(), 0);

    av_audio_fifo_read(audio_fifo_, (void **)temp_frame->data, enc_frame_size);

    temp_frame->pts = audio_pts_ctr_;
    audio_pts_ctr_ += enc_frame_size;

    int ret = avcodec_send_frame(audio_enc_ctx_.get(), temp_frame.get());
    if (ret >= 0) {
      while (avcodec_receive_packet(audio_enc_ctx_.get(), pkt.get()) >= 0) {
        av_packet_rescale_ts(pkt.get(), audio_enc_ctx_->time_base,
                             audio_out_stream_->time_base);
        pkt->stream_index = audio_out_stream_->index;
        av_interleaved_write_frame(fmt_ctx_.get(), pkt.get());
        av_packet_unref(pkt.get());
      }
    }
  }
}

void VideoTranscoder::FlushAudio() {
  int remaining = av_audio_fifo_size(audio_fifo_);
  if (remaining > 0) {
    auto temp_frame = make_unique_frame();
    auto pkt = make_unique_packet();

    temp_frame->nb_samples = remaining;
    temp_frame->format = audio_enc_ctx_->sample_fmt;
    temp_frame->sample_rate = audio_enc_ctx_->sample_rate;
    av_channel_layout_copy(&temp_frame->ch_layout, &audio_enc_ctx_->ch_layout);

    av_frame_get_buffer(temp_frame.get(), 0);
    av_audio_fifo_read(audio_fifo_, (void **)temp_frame->data, remaining);

    temp_frame->pts = audio_pts_ctr_;

    avcodec_send_frame(audio_enc_ctx_.get(), temp_frame.get());
    while (avcodec_receive_packet(audio_enc_ctx_.get(), pkt.get()) >= 0) {
      av_packet_rescale_ts(pkt.get(), audio_enc_ctx_->time_base,
                           audio_out_stream_->time_base);
      pkt->stream_index = audio_out_stream_->index;
      av_interleaved_write_frame(fmt_ctx_.get(), pkt.get());
      av_packet_unref(pkt.get());
    }
  }
}