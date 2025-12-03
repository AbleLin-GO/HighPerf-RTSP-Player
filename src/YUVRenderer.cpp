#include "../includes/YUVRenderer.h"
#include <iostream>
#include <vector>

// 顶点着色器 (不变)
const char *VERTEX_SHADER = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec2 aTexCoord;
    out vec2 TexCoord;
    void main() {
        gl_Position = vec4(aPos, 1.0);
        TexCoord = aTexCoord;
    }
)";

// 片元着色器 (不变)
const char *FRAG_SHADER = R"(
    #version 330 core
    in vec2 TexCoord;
    out vec4 FragColor;
    uniform sampler2D tex_y;
    uniform sampler2D tex_u;
    uniform sampler2D tex_v;
    void main() {
        float y = texture(tex_y, TexCoord).r - 0.0625;
        float u = texture(tex_u, TexCoord).r - 0.5;
        float v = texture(tex_v, TexCoord).r - 0.5;
        float r = y * 1.164 + v * 1.596;
        float g = y * 1.164 - u * 0.391 - v * 0.813;
        float b = y * 1.164 + u * 2.018;
        FragColor = vec4(r, g, b, 1.0);
    }
)";

YUVRenderer::YUVRenderer() {}

YUVRenderer::~YUVRenderer() {
  if (mContext)
    SDL_GL_DeleteContext(mContext);
  // 实际项目中还需要 glDeleteTextures 等清理工作
}

bool YUVRenderer::Init(SDL_Window *window, int width, int height) {
  mWindow = window;
  mVideoWidth = width;
  mVideoHeight = height;

  // 1. 创建 OpenGL 上下文 (接管窗口)
  // SDL_GL_CreateContext的作用：创建OpenGL上下文，关联到SDL窗口
  mContext = SDL_GL_CreateContext(window);
  if (!mContext) {
    std::cerr << "[OpenGL] Context creation failed: " << SDL_GetError()
              << std::endl;
    return false;
  }

  // 2. 初始化 GLEW
  // GLEW的作用：初始化OpenGL扩展库，确保使用最新的OpenGL功能
  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) {
    std::cerr << "[OpenGL] GLEW Init failed!" << std::endl;
    return false;
  }

  // 3. 编译 Shader'
  // CompileShader的作用：编译OpenGL着色器程序，返回程序ID
  mProgram = CompileShader(VERTEX_SHADER, FRAG_SHADER);
  if (mProgram == 0)
    return false;

  //  准备几何体 (矩形)
  float vertices[] = {
      // 位置             // 纹理坐标 (注意 Y 轴翻转，视频通常是左上角起始)
      -1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 1.0f,  1.0f,  0.0f, 1.0f, 0.0f,
      1.0f,  -1.0f, 0.0f, 1.0f, 1.0f, -1.0f, -1.0f, 0.0f, 0.0f, 1.0f};
  unsigned int indices[] = {0, 1, 2, 2, 3, 0};

  // 4. 创建 VAO, VBO, EBO
  // glGenVertexArrays的作用：生成顶点数组对象ID，用于存储顶点属性配置
  // glGenBuffers的作用：生成顶点缓冲对象ID，用于存储顶点数据
  // glGenBuffers的作用：生成索引缓冲对象ID，用于存储索引数据
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);

  // 5. 配置 VBO 和 EBO
  glBindVertexArray(VAO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
               GL_STATIC_DRAW);

  // 6. 设置顶点属性指针
  // glVertexAttribPointer的作用：指定顶点属性的布局，关联到VBO数据
  // glEnableVertexAttribArray的作用：启用顶点属性数组，使OpenGL知道如何使用该属性
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  // 7. 创建 3 个纹理对象
  // glGenTextures的作用：生成纹理对象ID，用于存储图像数据
  glGenTextures(3, mTextures);
  for (int i = 0; i < 3; i++) {
    glBindTexture(GL_TEXTURE_2D, mTextures[i]);
    // 关键参数：视频可能是变长的，需要允许修改
    // 各个参数意义：
    // GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE：纹理坐标超出 [0, 1] 范围时， clamp
    // 到边缘 GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE：与 S 轴相同，用于处理垂直方向
    // GL_TEXTURE_MIN_FILTER, GL_LINEAR：纹理缩小时使用线性插值
    // GL_TEXTURE_MAG_FILTER, GL_LINEAR：纹理放大时使用线性插值
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  }

  // 8. 绑定 Uniform
  // glUseProgram的作用：使用指定的着色器程序，后续的 OpenGL 调用将使用该程序
  // glGetUniformLocation的作用：获取着色器程序中指定 Uniform 变量的位置
  // glUniform1i的作用：设置指定 Uniform 变量的值，这里是设置纹理单元索引
  glUseProgram(mProgram);
  glUniform1i(glGetUniformLocation(mProgram, "tex_y"), 0);
  glUniform1i(glGetUniformLocation(mProgram, "tex_u"), 1);
  glUniform1i(glGetUniformLocation(mProgram, "tex_v"), 2);

  return true;
}

// 9. 渲染函数
// Render的作用：渲染一帧 YUV 视频，更新纹理数据
void YUVRenderer::Render(const uint8_t *dataY, const uint8_t *dataU,
                         const uint8_t *dataV, int linesizeY, int linesizeU,
                         int linesizeV) {

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  glUseProgram(mProgram);

  // 这里的核心逻辑是处理 linesize (stride) 对齐问题
  // FFmpeg 的 linesize 可能大于 width，OpenGl 默认 4 字节对齐，
  // 所以需要设置 GL_UNPACK_ROW_LENGTH 来告诉 OpenGL 每行数据的实际字节数
  // 这里设置为 linesizeY 是因为 Y 平面的 linesize 通常等于 width
  // 而 U/V 平面的 width 是 Y 的一半，linesize 也会相应减半
  glPixelStorei(GL_UNPACK_ROW_LENGTH, linesizeY);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, mTextures[0]);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, mVideoWidth, mVideoHeight, 0, GL_RED,
               GL_UNSIGNED_BYTE, dataY);

  glPixelStorei(GL_UNPACK_ROW_LENGTH,
                linesizeU); // U/V 宽度是一半，但 linesize 是解码器给的
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, mTextures[1]);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, mVideoWidth / 2, mVideoHeight / 2, 0,
               GL_RED, GL_UNSIGNED_BYTE, dataU);

  glPixelStorei(GL_UNPACK_ROW_LENGTH, linesizeV);
  glActiveTexture(GL_TEXTURE2);
  glBindTexture(GL_TEXTURE_2D, mTextures[2]);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, mVideoWidth / 2, mVideoHeight / 2, 0,
               GL_RED, GL_UNSIGNED_BYTE, dataV);

  glBindVertexArray(VAO);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

  SDL_GL_SwapWindow(mWindow);
}

// 在 src/YUVRenderer.cpp 末尾

GLuint YUVRenderer::CompileShader(const char *vertexSource,
                                  const char *fragmentSource) {
  // 1. 编译顶点着色器
  // glCreateShader的作用：创建一个新的着色器对象，返回其 ID
  // glShaderSource的作用：指定着色器的源代码
  // glCompileShader的作用：编译着色器代码
  GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShader, 1, &vertexSource, NULL);
  glCompileShader(vertexShader);

  // 检查错误
  GLint success;
  char infoLog[512];
  glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
    std::cerr << "[OpenGL] Vertex Shader Error: " << infoLog << std::endl;
    return 0;
  }

  // 2. 编译片元着色器
  GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShader, 1, &fragmentSource, NULL);
  glCompileShader(fragmentShader);

  // 检查错误
  glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
    std::cerr << "[OpenGL] Fragment Shader Error: " << infoLog << std::endl;
    return 0;
  }

  // 3. 链接程序
  GLuint shaderProgram = glCreateProgram();
  glAttachShader(shaderProgram, vertexShader);
  glAttachShader(shaderProgram, fragmentShader);
  glLinkProgram(shaderProgram);

  // 检查错误
  glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
    std::cerr << "[OpenGL] Shader Link Error: " << infoLog << std::endl;
    return 0;
  }

  // 4. 清理
  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  return shaderProgram;
}