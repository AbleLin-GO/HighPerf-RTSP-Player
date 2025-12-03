// includes/AVRAII.h

#pragma once

#include <memory>
// 引入所有需要的 FFmpeg C 库
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/packet.h>
#include <libavfilter/avfilter.h>
#include <libavformat/avformat.h>
#include <libavutil/frame.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}

// ----------------------------------------
// AVPacket 的 Deleter 和 UniquePtr
// ----------------------------------------
struct AVPacketDeleter {
  void operator()(AVPacket *packet) const {
    if (packet) {
      av_packet_free(&packet);
    }
  }
};
using UniqueAVPacket = std::unique_ptr<AVPacket, AVPacketDeleter>;

// ----------------------------------------
// AVFrame 的 Deleter 和 UniquePtr
// ----------------------------------------
struct AVFrameDeleter {
  void operator()(AVFrame *frame) const {
    if (frame) {
      av_frame_free(&frame);
    }
  }
};
using UniqueAVFrame = std::unique_ptr<AVFrame, AVFrameDeleter>;

// ... (你可以将所有之前定义的 UniquePtr 类型都放在这里) ...

// ----------------------------------------
// AVFormatContext 的 Deleter 和 UniquePtr
// ----------------------------------------
struct AVFormatContextDeleter {
  void operator()(AVFormatContext *ctx) const {
    if (ctx) {
      avformat_close_input(&ctx);
    }
  }
};
using UniqueAVFormatContext =
    std::unique_ptr<AVFormatContext, AVFormatContextDeleter>;
// ------------------------------

struct AVCodecContextDeleter {
  void operator()(AVCodecContext *ctx) const {
    if (ctx) {
      avcodec_free_context(&ctx);
    }
  }
};
using UniqueAVCodecContext =
    std::unique_ptr<AVCodecContext, AVCodecContextDeleter>;

struct SwsContextDeleter {
  void operator()(SwsContext *ctx) const {
    if (ctx) {
      sws_freeContext(ctx);
    }
  }
};
using UniqueSwsContext = std::unique_ptr<SwsContext, SwsContextDeleter>;

struct SwrContextDeleter {
  void operator()(SwrContext *ctx) const {
    if (ctx) {
      swr_free(&ctx);
    }
  }
};
using UniqueSwrContext = std::unique_ptr<SwrContext, SwrContextDeleter>;

// ----------------------------------------
// AVFilterGraph 的 Deleter 和 UniquePtr
// ----------------------------------------
struct AVFilterGraphDeleter {
  void operator()(AVFilterGraph *graph) const {
    if (graph) {
      avfilter_graph_free(&graph);
    }
  }
};
using UniqueAVFilterGraph =
    std::unique_ptr<AVFilterGraph, AVFilterGraphDeleter>;

// [新增] 针对输出(Muxing)的 Context Deleter
struct AVOutputFormatContextDeleter {
  void operator()(AVFormatContext *ctx) const {
    if (ctx) {
      // 输出上下文只需要释放内存，IO 关闭通常由 avio_closep 手动管理
      avformat_free_context(ctx);
    }
  }
};
using UniqueAVOutputFormatContext =
    std::unique_ptr<AVFormatContext, AVOutputFormatContextDeleter>;

// ----------------------------------------
//  工厂辅助函数 (Factory Functions)
// ----------------------------------------

// 修复报错: Use of undeclared identifier 'make_unique_packet'
inline UniqueAVPacket make_unique_packet() {
  return UniqueAVPacket(av_packet_alloc());
}

// 修复报错: Use of undeclared identifier 'make_unique_frame'
inline UniqueAVFrame make_unique_frame() {
  return UniqueAVFrame(av_frame_alloc());
}

// ==========================================
//  Flush Packet 工厂函数
// ==========================================
// 创建一个特殊的 Flush Packet，用于 Seek 时通知解码器清空缓存
// 我们约定：stream_index = -1 代表 Flush 包
inline UniqueAVPacket make_flush_packet() {
  auto pkt = make_unique_packet();
  pkt->data = nullptr;
  pkt->size = 0;
  pkt->stream_index = -1; // 特殊标记
  return pkt;
}
