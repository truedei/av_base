#ifndef PUSH_STREAM_FFMPEG_H
#define PUSH_STREAM_FFMPEG_H

#include "main.h"
#include "iostream"

#include "librtmp/rtmp.h"

using namespace std;

#define ERROR_BUF 1024

class PushStreamFFmpeg
{
private:
    /* data */

    char *rtmpAddress;
    char *flvAddress;

    AVFormatContext *octx = NULL;
    
    char errorBuf[ERROR_BUF] = {0,};

    // 推流的每一帧数据
    AVPacket avPacket;

    //开始推流的时间
    int64_t start_time_ = 0;


    RTMP *rtmp = NULL;


public:
    PushStreamFFmpeg(/* args */);
    ~PushStreamFFmpeg();

    // 设置rtmp地址
    void SetRTMPAddress( char *rtmpAddress);
    // 获取推流的rtmp地址
     char *GetRTMPAddress();




    // 初始化参数
    int InitPushStreamPareams();

    // 采集发送数据
    void SendData(char *data, int size);

    // 初始化设备
    int InitDevice();

    // 获取当前的时间
    int64_t GetCurrentTimeMsec()
    {
        return av_gettime();
    }

    // 获取pts时间
    int64_t GetVideoPts()
    {
        int64_t pts = GetCurrentTimeMsec() - start_time_;
        //可以做时间矫正

        return pts;
    }
    // 获取dts时间
    int64_t GetVideoDts()
    {
        // 如果有需要的话，时间可以不一致
        return GetVideoPts();
    }

    //librtmp相关
    
    // 设置flv文件地址
    void SetFLVAddress( char *flvAddress);
    // 获取flv文件的地址
     char *GetFLVAddress();


    // 初始化librtmp相关参数
    int InitLibRTMPPareams();
    //关闭相关资源
    void CloseLibtRTMPPareams();
    //开始发送数据
    void StartSendData();

    int sendPacket(unsigned int packet_type, unsigned char *data,
                           unsigned int size, unsigned int timestamp);




};

enum RTMPChannel
{
   RTMP_NETWORK_CHANNEL = 2,   ///< channel for network-related messages (bandwidth report, ping, etc)
   RTMP_SYSTEM_CHANNEL,        ///< channel for sending server control messages
   RTMP_AUDIO_CHANNEL,         ///< channel for audio data
   RTMP_VIDEO_CHANNEL   = 6,   ///< channel for video data
   RTMP_SOURCE_CHANNEL  = 8,   ///< channel for a/v invokes
};



#endif
