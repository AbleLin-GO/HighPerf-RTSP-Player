cmake_minimum_required(VERSION 3.10)
project(FFmpegPlayer)

# 查找系统库
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED libavformat libavcodec libavutil libavfilter)

find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# 添加头文件路径
include_directories(includes)
include_directories(${FFMPEG_INCLUDE_DIRS}) # <-- 确保包含 FFmpeg 头文件
include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})


# -------------------- III. 模块化库构建 --------------------

# 手动指定源文件给各自的库
set(PACKET_QUEUE_SRC src/PacketQueue.cpp)
set(FRAME_QUEUE_SRC src/FrameQueue.cpp)
set(PLAYER_STATE_SRC src/PlayerState.cpp src/VideoTranscoder.cpp) 
set(RENDERER_SRC src/YUVRenderer.cpp)
set(MAIN_SRC src/main.cpp)

# 1. 创建 PacketQueue 静态库
add_library(PacketQueue_Lib STATIC ${PACKET_QUEUE_SRC})
# 确保库链接到它们需要的 C 库
target_link_libraries(PacketQueue_Lib PRIVATE ${FFMPEG_LIBRARIES} Threads::Threads)

# 2. 创建 FrameQueue 静态库
add_library(FrameQueue_Lib STATIC ${FRAME_QUEUE_SRC})
target_link_libraries(FrameQueue_Lib PRIVATE ${FFMPEG_LIBRARIES} Threads::Threads)

# 3. 创建 PlayerState 静态库 
add_library(PlayerState_Lib STATIC ${PLAYER_STATE_SRC})
target_link_libraries(PlayerState_Lib PRIVATE ${FFMPEG_LIBRARIES} Threads::Threads)

# 4.新增：创建 Renderer 静态库 #
add_library(Renderer_Lib STATIC ${RENDERER_SRC})
# Renderer 需要链接 SDL2 (用于窗口) 和 OpenGL/GLEW
target_link_libraries(Renderer_Lib PRIVATE ${SDL2_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES})


# -------------------- IV. 可执行文件构建 --------------------
# 1. 创建可执行文件
add_executable(player_exe ${MAIN_SRC})

# 2. 链接所有依赖
target_link_libraries(
    player_exe 
    PRIVATE 
    -Wl,--start-group 
    PacketQueue_Lib         
    FrameQueue_Lib          
    PlayerState_Lib       
    Renderer_Lib  
    ${FFMPEG_LIBRARIES}    

    -lswresample
    -lswscale
    -lavformat
    -lavcodec
    -lavutil
    -Wl,--end-group

    ${SDL2_LIBRARIES} 
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}

    Threads::Threads
    -lm
)

# 设置输出目录
set_target_properties(player_exe PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")