#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <SDL2/SDL_video.h>
#include <iostream>
#include <vector>
#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>

// 顶点着色器代码（决定画在哪）
const char *vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec2 aTexCoord; // 接收纹理坐标

    out vec2 TexCoord; // 输出到片元着色器的纹理坐标

    void main()
    {
        gl_Position = vec4(aPos, 1.0);
        TexCoord = aTexCoord; // 将纹理坐标传递给片元着色器
    }
)";

// 片元着色器代码（决定画什么）
const char *fragmentShaderSource = R"(
    #version 330 core
    in vec2 TexCoord; // 接收从顶点着色器传递的纹理坐标
    out vec4 FragColor;

    // 新增三个采样器
    uniform sampler2D tex_y;
    uniform sampler2D tex_u;
    uniform sampler2D tex_v;
    void main()
    {
// 1. 从三个纹理中采样 Y, U, V 的值
        // texture() 函数读取对应坐标的纹理像素
        // Y 是亮度， U, V 是色度。YUV 的值域通常是 [0, 1] 归一化
        float y = texture(tex_y, TexCoord).r;
        float u = texture(tex_u, TexCoord).r;
        float v = texture(tex_v, TexCoord).r;

        // 2. 偏移 (将 YUV 的范围 [16, 235], [16, 240] 调整到 [0, 1] 对应的线性范围)
        // BT.709 标准偏移量：
        y = y - 0.0625; // Y' = Y - 16/256
        u = u - 0.5;    // U' = U - 128/256
        v = v - 0.5;    // V' = V - 128/256

        // 3. YUV -> RGB 转换矩阵 (BT.709/BT.601 修正版)
        float r = y * 1.164 + v * 1.596;
        float g = y * 1.164 - u * 0.391 - v * 0.813;
        float b = y * 1.164 + u * 2.018;

        // 4. 输出颜色
        FragColor = vec4(r, g, b, 1.0);
    }
)";

int main() {
  SDL_Init(SDL_INIT_VIDEO);

  // 设置OPENGL版本属性
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

  // 创建支持 OpenGL 的窗口
  // 参数：窗口标题、窗口位置、窗口大小、窗口标志
  SDL_Window *window = SDL_CreateWindow("OpenGL Test", SDL_WINDOWPOS_CENTERED,
                                        SDL_WINDOWPOS_CENTERED, 800, 600,
                                        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);

  // 创建 OpenGL 上下文
  SDL_GLContext glContext = SDL_GL_CreateContext(window);

  // GLEW初始化
  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) {
    std::cerr << "GLEW初始化失败" << std::endl;
    return -1;
  }
  // 打印显卡的具体信息：供应商、渲染器
  std::cout << "[gpu]VENDOR:" << glGetString(GL_VENDOR) << std::endl;
  std::cout << "[gpu]RENDERER:" << glGetString(GL_RENDERER) << std::endl;

  // 编译着色器
  // 创建顶点着色器对象
  unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
  // 绑定顶点着色器代码到着色器对象
  glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
  // 编译顶点着色器
  glCompileShader(vertexShader);

  // 创建片元着色器对象
  unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  // 绑定片元着色器代码到着色器对象
  glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
  // 编译片元着色器
  glCompileShader(fragmentShader);

  // 创建着色器程序对象
  unsigned int shaderProgram = glCreateProgram();
  //  Attach 顶点着色器和片元着色器到程序对象
  glAttachShader(shaderProgram, vertexShader);
  glAttachShader(shaderProgram, fragmentShader);
  // 链接程序对象
  glLinkProgram(shaderProgram);
  // 删除着色器对象，因为它们已经链接到程序对象了
  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  // --- 纹理和 uniform 变量设置 ---
  GLuint textures[3];
  glGenTextures(3, textures);

  // 获取 Shader 中 uniform 变量的位置
  glUseProgram(shaderProgram);
  glUniform1i(glGetUniformLocation(shaderProgram, "tex_y"), 0); // 纹理单元 0
  glUniform1i(glGetUniformLocation(shaderProgram, "tex_u"), 1); // 纹理单元 1
  glUniform1i(glGetUniformLocation(shaderProgram, "tex_v"), 2); // 纹理单元 2
  glUseProgram(0); // 停止使用 Program

  // --- 模拟数据尺寸 (用作初始化) ---
  const int VIDEO_W = 640;
  const int VIDEO_H = 480;

  // Y, U, V 数据指针 (这里只是占位符，实际来自 AVFrame)
  // Y data size: 640*480 = 307200 bytes
  // U/V data size: 320*240 = 76800 bytes each
  std::vector<uint8_t> dummy_y(VIDEO_W * VIDEO_H, 128);     // 灰色 Y
  std::vector<uint8_t> dummy_u(VIDEO_W * VIDEO_H / 4, 128); // 灰色 U/V
  std::vector<uint8_t> dummy_v(VIDEO_W * VIDEO_H / 4, 128);
  // (用 128 填充，Y=128是灰色，U/V=128是无色差)

  // --- 初始分配 GPU 内存空间并绑定参数  ---

  // 1. Y 分量 (全尺寸)
  glActiveTexture(GL_TEXTURE0);              // 激活纹理单元 0
  glBindTexture(GL_TEXTURE_2D, textures[0]); // 绑定纹理 ID
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  // 核心：GL_RED 格式，NULL 数据占位，分配 GPU 内存
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, VIDEO_W, VIDEO_H, 0, GL_RED,
               GL_UNSIGNED_BYTE, NULL);

  // 2. U 分量 (四分之一尺寸)
  glActiveTexture(GL_TEXTURE1); // 激活纹理单元 1
  glBindTexture(GL_TEXTURE_2D, textures[1]);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  // 核心：注意尺寸 W/2, H/2
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, VIDEO_W / 2, VIDEO_H / 2, 0, GL_RED,
               GL_UNSIGNED_BYTE, NULL);

  // 3. V 分量 (四分之一尺寸)
  glActiveTexture(GL_TEXTURE2); // 激活纹理单元 2
  glBindTexture(GL_TEXTURE_2D, textures[2]);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, VIDEO_W / 2, VIDEO_H / 2, 0, GL_RED,
               GL_UNSIGNED_BYTE, NULL);

  // --- 替换数据 (一个覆盖全屏的矩形，并带纹理坐标) ---
  float vertices[] = {
      // 顶点坐标 (Position: x, y, z)   // 纹理坐标 (TexCoord: u, v)
      -1.0f, 1.0f,  0.0f, 0.0f, 0.0f, // 0: Top Left (屏幕左上角)
      1.0f,  1.0f,  0.0f, 1.0f, 0.0f, // 1: Top Right (屏幕右上角)
      1.0f,  -1.0f, 0.0f, 1.0f, 1.0f, // 2: Bottom Right (屏幕右下角)
      -1.0f, -1.0f, 0.0f, 0.0f, 1.0f  // 3: Bottom Left (屏幕左下角)
  };

  // 索引 (Indices) 用于复用顶点，绘制两个三角形组成一个矩形
  unsigned int indices[] = {
      0, 1, 2, // 第一个三角形
      2, 3, 0  // 第二个三角形
  };

  // 上传到GPU（创建顶点数组对象和顶点缓冲对象）
  unsigned int VBO, VAO, EBO;
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);
  // 绑定VAO
  glBindVertexArray(VAO);
  // 绑定VBO并上传数据
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  // 绑定EBO并上传索引数据
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
               GL_STATIC_DRAW);

  // 位置属性：位置 (location 0) 是前 3 个 float
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
  // 启用顶点属性数组 (0 是位置属性)
  glEnableVertexAttribArray(0);
  // 纹理坐标属性 (1 是纹理坐标属性)
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                        (void *)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  // --- 渲染循环 ---
  bool quit = false;
  SDL_Event e;
  while (!quit) {
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT)
        quit = true;
    }
    // --- 替换渲染循环中的绘制逻辑 ---

    // 1. 清屏
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // 2. 使用Shader
    glUseProgram(shaderProgram);

    // 3. **【核心】更新纹理数据 (模拟 FFmpeg 帧更新)**
    // 注意：使用 glTexSubImage2D 才能实现高效的 GPU 数据更新

    // 3.1 更新 Y 分量
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textures[0]);
    // 使用 SubImage 更新数据，速度远高于 glTexImage2D
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, VIDEO_W, VIDEO_H, GL_RED,
                    GL_UNSIGNED_BYTE, dummy_y.data());

    // 3.2 更新 U 分量
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, textures[1]);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, VIDEO_W / 2, VIDEO_H / 2, GL_RED,
                    GL_UNSIGNED_BYTE, dummy_u.data());

    // 3.3 更新 V 分量
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, textures[2]);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, VIDEO_W / 2, VIDEO_H / 2, GL_RED,
                    GL_UNSIGNED_BYTE, dummy_v.data());

    // 4. 绘制矩形
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    // 5. 交换缓冲区 (双缓冲)
    SDL_GL_SwapWindow(window);
  }

  return 0;
}
