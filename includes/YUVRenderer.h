#pragma once
#include <GL/glew.h> // 必须在 SDL 之前
#include <SDL2/SDL.h>

class YUVRenderer {
public:
  YUVRenderer();
  ~YUVRenderer();

  // 初始化：传入 SDL 窗口，建立 OpenGL 上下文
  bool Init(SDL_Window *window, int videoWidth, int videoHeight);

  // 核心渲染：传入解码后的 AVFrame，画到屏幕上
  // 替换掉了以前的 SDL_UpdateTexture + SDL_RenderCopy
  void Render(const uint8_t *dataY, const uint8_t *dataU, const uint8_t *dataV,
              int linesizeY, int linesizeU, int linesizeV);

private:
  SDL_Window *mWindow = nullptr;
  SDL_GLContext mContext = nullptr;

  GLuint mProgram = 0;
  GLuint mTextures[3] = {0}; // Y, U, V
  GLuint VAO = 0, VBO = 0, EBO = 0;

  int mVideoWidth = 0;
  int mVideoHeight = 0;

  // 编译 Shader 的辅助函数
  GLuint CompileShader(const char *vertexSource, const char *fragmentSource);
};