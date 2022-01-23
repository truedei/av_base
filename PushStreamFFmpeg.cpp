
#include "PushStreamFFmpeg.h"


PushStreamFFmpeg::PushStreamFFmpeg(/* args */)
{
}

PushStreamFFmpeg::~PushStreamFFmpeg()
{
}

// 设置rtmp地址
void PushStreamFFmpeg::SetRTMPAddress( char *rtmpAddress){
    this->rtmpAddress = rtmpAddress;
}

// 获取推流的rtmp地址
 char *PushStreamFFmpeg::GetRTMPAddress(){
    return this->rtmpAddress;
}


// 设置flv文件地址
void PushStreamFFmpeg::SetFLVAddress( char *flvAddress){
  this->flvAddress = flvAddress;
}

// 获取flv文件的地址
 char *PushStreamFFmpeg::GetFLVAddress(){
  return this->flvAddress;
}

/**
 * 
Input #0, flv, from '/data/project/VSCProject/ffmpegStudy/1480996582.flv':
  Metadata:
    encoder         : Lavf57.56.101
  Duration: 00:10:14.28, start: 0.000000, bitrate: 1008 kb/s
  Stream #0:0: Video: h264 (Main), yuv420p(progressive), 1280x720, 25 fps, 25 tbr, 1k tbn

avformat_alloc_output_context2 success!
Output #0, flv, to 'rtmp://wiki.truedei.com/live/room':
  Stream #0:0: Video: h264 (Main), yuv420p(progressive), 1280x720, q=2-31
avformat_write_header Success!
 * 
 * @return int 
 */
// 初始化参数
int PushStreamFFmpeg::InitPushStreamPareams(){

    /*********************
     * 输出流处理部分
     ********************/
    int ret = 0;

    // 1、初始化网络库
    avformat_network_init();

    // 2、初始化输出上喜爱问
    // 如果是输入文件，flv可以不传，可以从文件中判断。如果是流则必须传
    // 创建输出上下文
    ret = avformat_alloc_output_context2(&this->octx, NULL,"flv", this->GetRTMPAddress() );
    if(ret < 0)
    {
        av_strerror(ret ,errorBuf ,ERROR_BUF);
        printf("avformat_alloc_output_context2 error  ! errorBuf=%s \n" ,errorBuf);
        return ret;
    }
    cout << "avformat_alloc_output_context2 success!" << endl;

    // 创建新的流 --> 视频流
    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    AVStream *out = avformat_new_stream(this->octx ,codec);

    // 复制配置信息，用于mp4
    // AVCodecParameters *codecpar;
    
    out->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;//编码类型
    out->codecpar->codec_id = AV_CODEC_ID_H264;//编码格式
    out->codecpar->format = 0;//0:yuv420p
    out->codecpar->width = 1280;
    out->codecpar->height = 720;

    // 创建音频流
    // const AVCodec *audioCodec = avcodec_find_decoder(AV_CODEC_ID_AAC);
    // AVStream *audioOut = avformat_new_stream(this->octx ,audioCodec);
    // audioOut->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    // audioOut->codecpar->codec_id = AV_CODEC_ID_AAC;
    // audioOut->codecpar->channels = 2;
    // audioOut->codecpar->sample_rate = 42000;
    // audioOut->codecpar->channel_layout = 2;


    // 打印信息
    av_dump_format(this->octx, 0, this->GetRTMPAddress(), 1);


     /****************
     * 准备推流
     ***************/

    // 打开IO
    ret = avio_open(&octx->pb,this->GetRTMPAddress(),AVIO_FLAG_WRITE);
    if(ret < 0)
    {
        av_strerror(ret , errorBuf ,ERROR_BUF_LEN);
        printf("打开IO失败 ! %s \n",errorBuf);
        return ret;
    }

    // 写入头部信息
    ret = avformat_write_header(octx,0);
    if(ret < 0)
    {
        printf("写入头部信息失败 !\n");
        return ret;
    }
    cout << "avformat_write_header Success!" << endl;

    cout << this->octx->streams[0]->time_base.num << endl;
    cout << this->octx->streams[0]->time_base.den << endl;


    //记录开始时间
    start_time_ = av_gettime();

    return 0;
}

// 初始化设备
int PushStreamFFmpeg::InitDevice(){
    // AVFormatContext *pFmtCtx = avformat_alloc_context();  
    // AVDeviceInfoList *device_info = NULL;  
    // AVDictionary* options = NULL;  
    // av_dict_set(&options, "list_devices", "true", 0);  
    // const AVInputFormat *iformat = av_find_input_format("alsa");  
    // printf("Device Info=============\n");  
    // avformat_open_input(&pFmtCtx, "video=dummy", iformat, &options);  
    // printf("========================\n");  


    //Register Device  
    avdevice_register_all();

    // char* input_name= "video4linux2";  
    char* input_name= "1080p.h264";  
   
    char* file_name = "/dev/video0";  

    char* out_file  = "./test.jpeg";   

    AVFormatContext *fmtCtx = NULL;      
    AVPacket *packet;   
    const AVInputFormat *inputFmt;  
    FILE *fp;   
    int ret;  
    
    inputFmt = av_find_input_format (input_name);      
     
    if (inputFmt == NULL)    {          
        printf("can not find_input_format\n");          
        return -1;      
    }
    ret = avformat_open_input ( &fmtCtx, file_name, inputFmt, NULL);

    if ( ret < 0){  
       av_strerror(ret ,errorBuf ,ERROR_BUF_LEN);

       printf("can not open_input_file --> %s \n", errorBuf);         
       return -1;      
    }  

    /* print device information*/  
    av_dump_format(fmtCtx, 0, file_name, 0);  


    packet = (AVPacket *)av_malloc(sizeof(AVPacket));      
    av_read_frame(fmtCtx, packet);   
    printf("data length = %d\n",packet->size);     
  
    fp = fopen(out_file, "wb");      
    if (fp < 0)    {          
        printf("open frame data file failed\n");          
        return -1;      
    }      
      
    fwrite(packet->data, 1, packet->size, fp);      
  
    fclose(fp);      
    // av_free_packet(packet);      
    avformat_close_input(&fmtCtx);  

    return 0;
}

// 采集发送数据
void PushStreamFFmpeg::SendData(char *data, int size){


    int ret = 0;

    if(data == NULL || size <= 0)
    {
      return;
    }

    // 推流每一帧数据

    //封装avPacket
    AVPacket avPacket;

    // 获取当前的时间戳 微妙
    // start_time_ = av_gettime();
    sleep(2);

    //计算转换时间戳 pts dts
    // 获取时间基数
    avPacket.pts = this->GetVideoPts();
    avPacket.dts = this->GetVideoDts();
    
    //到这一帧时候经历了多长时间
    // avPacket.duration = av_rescale_q_rnd(avPacket.duration, itime, otime, (AVRounding)(AV_ROUND_NEAR_INF | AV_ROUND_NEAR_INF));
    avPacket.pos = -1;

    cout << "video pts=" << avPacket.pts << " " << endl;
    cout << "video dts=" << avPacket.dts << " " << endl;


    //推送  会自动释放空间 不需要调用av_packet_unref
    // ret = av_interleaved_write_frame(octx, &avPacket);
    // if (ret < 0) {

    //     return;
    // }
    //视频帧推送速度
    //if (avPacket.stream_index == 0)
    //  av_usleep(30 * 1000);
    //释放空间。内部指向的视频空间和音频空间
    //av_packet_unref(&avPacket);

}




/***************************
 * 
 * librtmp库相关
 * 
 *************************/

/**
 * @brief 初始化librtmp相关参数和对象，与rtmp服务器建立连接
 * 
 * @return int 
 */
int PushStreamFFmpeg::InitLibRTMPPareams(){
  //  this->rtmp = connect_rtmp_server((char *)this->rtmpAddress);

    //1、创建RTMP对象
    rtmp = RTMP_Alloc();
    if(!rtmp)
    {
        printf("RMTP对象分配失败! \n");
        this->CloseLibtRTMPPareams();
        return 0;
    }

    RTMP_Init(rtmp);


    //2、先设置RTMP服务器地址，以及连接的超时时间
    rtmp->Link.timeout = 10;
    RTMP_SetupURL(rtmp,this->GetRTMPAddress());


    //3、设置推流还是拉流
    //如果不设置就是拉流
    RTMP_EnableWrite(rtmp);


    //4、建立连接

    if(!RTMP_Connect(rtmp,NULL))
    {
        printf("建立连接失败! \n");
        this->CloseLibtRTMPPareams();
        return 0;
    }

    //5、创建流
    RTMP_ConnectStream(rtmp,0);

    printf("建立rtmp连接成功! \n");
    return 0;
}

//关闭相关资源
void PushStreamFFmpeg::CloseLibtRTMPPareams(){
    //释放资源
    if(rtmp)
    {
        RTMP_Close(rtmp);
        RTMP_Free(rtmp);
    }
}



//开始发送数据
void PushStreamFFmpeg::StartSendData(){

  /*****************************
   * 输入流处理部分
   ****************************/
    
    int ret = 0;

    // 对文件解封装
    AVFormatContext *ictx = NULL;

    // 打开文件，解封文件头
    ret = avformat_open_input(&ictx, this->GetFLVAddress(), 0, NULL);

    if(ret < 0 ){
        printf("打开文件失败 ! \n");;
        return;
    }

    // 获取音视频的信息， .h264 flv没有头信息
    ret = avformat_find_stream_info(ictx,0);
    if(ret != 0)
    {
        printf("avformat_find_stream_info error !\n");
        return;
    }

    // 打印信息
    //0：打印所有 
    av_dump_format(ictx, 0, this->GetFLVAddress(), 0);

    int j = 0;
    int videoindex = 0;
    int audioindex = 0;
    for(j = 0;j < ictx->nb_streams ;j++)
    {
        if (ictx->streams[j]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
        {
            videoindex = j;
        }
        else if (ictx->streams[j]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO)
        {
            audioindex = j;
        }
    }

    if(videoindex < 0)
    {
        printf("没有找到视频流 videoindex=%d  \n ",videoindex);
    }

    if(audioindex < 0)
    {
        printf("没有找到音频流 audioindex=%d  \n ",audioindex);
    }

    if(videoindex < 0 && audioindex < 0)
    {
      printf("视频流和音频流都没找到 \n");
      return;
    }

    /**************************************
     * 
     * 读取数据
     * 
     ****************************************/

    // 推流每一帧数据
    AVPacket avPacket;

    while(1)
    {
        ret = av_read_frame(ictx, &avPacket);
        if(ret < 0){
          printf("结束了! \n");
          break;
        }

        // printf("avPacket.pts = %d,avPacket.data =%d,avPacket.size=%d,time_base=%d  \n",avPacket.pts,avPacket.data ,avPacket.size,av_q2d(avPacket.time_base));

        printf("%d ",avPacket.pts);

        // 发送数据
        //如果流是视频
        if(avPacket.stream_index = videoindex)
        {
            // this->sendPacket(RTMP_PACKET_TYPE_VIDEO ,avPacket.data ,avPacket.size,av_q2d(avPacket.time_base));
            this->sendPacket(RTMP_PACKET_TYPE_VIDEO ,avPacket.data ,avPacket.size, avPacket.pts);
        }

        usleep(100000);
        // av_packet_free(&avPacket);
        av_packet_unref(&avPacket);
    }

}


/**
 * @brief 发送数据
 * 
 * @param packet_type 
 * @param data 
 * @param size 
 * @param timestamp 
 * @return int 
 */
int PushStreamFFmpeg::sendPacket(unsigned int packet_type, unsigned char *data,
                           unsigned int size, unsigned int timestamp)
{
    if (rtmp == NULL)
    {
        return FALSE;
    }

    RTMPPacket packet;
    RTMPPacket_Reset(&packet);
    RTMPPacket_Alloc(&packet, size);

    packet.m_packetType = packet_type;
    if(packet_type == RTMP_PACKET_TYPE_AUDIO)
    {
        packet.m_nChannel = RTMP_AUDIO_CHANNEL;
        //               LogInfo("audio packet timestamp:%u", timestamp);
    }
    else if(packet_type == RTMP_PACKET_TYPE_VIDEO)
    {
        packet.m_nChannel = RTMP_VIDEO_CHANNEL;
//                      LogInfo("video packet timestamp:%u, size:%u", timestamp, size);
    }
    else
    {
        packet.m_nChannel = RTMP_NETWORK_CHANNEL;
    }
     packet.m_nChannel = RTMP_AUDIO_CHANNEL;
    packet.m_headerType = RTMP_PACKET_SIZE_LARGE;
    packet.m_nTimeStamp = timestamp;
    packet.m_nInfoField2 = rtmp->m_stream_id;
    packet.m_nBodySize = size;
    memcpy(packet.m_body, data, size);

    int nRet = RTMP_SendPacket(rtmp, &packet, 0);
    if (nRet != 1)
    {
        printf("RTMP_SendPacket fail %d\n",nRet);
    }

    RTMPPacket_Free(&packet);

    return nRet;
}



