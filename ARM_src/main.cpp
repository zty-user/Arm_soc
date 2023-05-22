//////////////////////////////////////////////////////////////////
//  复旦大学2019-2020学年秋季学期“系统级可编程芯片设计”课程期末Project
//  无人值守停车场的收费关卡系统
//  Team : 任予琛 + 靳子路 + 刘思敏
//  Written by : 任予琛
//  Date : 2019.12
//////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "hps_0.h"
#include "plr.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>           
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <asm/types.h>        
#include <linux/videodev2.h>
#include <ctime>

#define CAMERA_DEVICE "/dev/video0"
#define CAPTURE_FILE  "capture.bmp"

#define VIDEO_WIDTH  640
#define VIDEO_HEIGHT 480
#define VIDEO_FORMAT V4L2_PIX_FMT_YUYV  //UYVY:422
#define BUFFER_COUNT 1


//////////////////////////////////////////////////////////////////////////
//         UYVY422 -> RGB -> BMP24
//////////////////////////////////////////////////////////////////////////

#define TUNE(r) ((r) < 0 ? 0 : ((r) > 255 ? 255 : (r)))
clock_t t_start, t_finish;
bool flag;
//Reference : https://blog.csdn.net/qq_29350001/article/details/52032540
//Modified by Ren Yuchen
void uyvy422_to_rgb(unsigned char* pYUV, unsigned char* pRGB)   {
    unsigned char y, u, v, y1;
    double r, g, b;
    unsigned int loop = (VIDEO_WIDTH * VIDEO_HEIGHT) >> 1;

    while(loop-- > 0)  {
        y  = *pYUV; pYUV++;
        u  = *pYUV; pYUV++;
        y1  = *pYUV; pYUV++;
        v = *pYUV; pYUV++;

        r = 1.164 * (y - 16) + 1.596 * (v - 128);
        g = 1.164 * (y - 16) - 0.813 * (v - 128) - 0.392 * (u - 128);
        b = 1.164 * (y - 16) + 2.017 * (u - 128);

        *pRGB = TUNE(b); pRGB++;
        *pRGB = TUNE(g); pRGB++;
        *pRGB = TUNE(r); pRGB++;
        
        r = 1.164 * (y1 - 16) + 1.596 * (v - 128);
        g = 1.164 * (y1 - 16) - 0.813 * (v - 128) - 0.392 * (u - 128);
        b = 1.164 * (y1 - 16) + 2.017 * (u - 128);

        *pRGB = TUNE(b); pRGB++;
        *pRGB = TUNE(g); pRGB++;
        *pRGB = TUNE(r); pRGB++;
    }
}

typedef long LONG;
typedef unsigned long DWORD;
typedef unsigned short WORD;

typedef struct {
        WORD   bfType;
        DWORD  bfSize;
        WORD   bfReserved1;
        WORD   bfReserved2;
        DWORD  bfOffBits;
} BMPFILEHEADER_T;
 
typedef struct{
        DWORD  biSize;
        LONG   biWidth;
        LONG   biHeight;
        WORD   biPlanes;
        WORD   biBitCount;
        DWORD  biCompression;
        DWORD  biSizeImage;
        LONG   biXPelsPerMeter;
        LONG   biYPelsPerMeter;
        DWORD  biClrUsed;
        DWORD  biClrImportant;
} BMPINFOHEADER_T;

//Reference : https://blog.csdn.net/u014568921/article/details/48369869
//Modified by Ren Yuchen
int rgb_to_bmp(unsigned char* pdata)   {
    int size = VIDEO_WIDTH * VIDEO_HEIGHT * 3 * sizeof(char);

    BMPFILEHEADER_T bfh;
    bfh.bfType = (WORD)0x4d42;
    bfh.bfSize = 14;	//size + sizeof(BMPFILEHEADER_T) + sizeof(BMPINFOHEADER_T);
    bfh.bfReserved1 = 0; 
    bfh.bfReserved2 = 0; 
    bfh.bfOffBits = 54;	//sizeof(BMPFILEHEADER_T) + sizeof(BMPINFOHEADER_T);
 
    BMPINFOHEADER_T bih;
    bih.biSize = sizeof(BMPINFOHEADER_T);
    bih.biWidth = VIDEO_WIDTH;
    bih.biHeight = -VIDEO_HEIGHT;
    bih.biPlanes = 1;
    bih.biBitCount = 24;
    bih.biCompression = 0;
    bih.biSizeImage = size;
    bih.biXPelsPerMeter = 0;
    bih.biYPelsPerMeter = 0;
    bih.biClrUsed = 0;
    bih.biClrImportant = 0;
    FILE * fp;
    fp = fopen(CAPTURE_FILE, "wb");
/*
	switch (loop)	{
		case 8 : fp = fopen("8.bmp", "wb"); break;
		case 7 : fp = fopen("7.bmp", "wb"); break;
		case 6 : fp = fopen("6.bmp", "wb"); break;
		case 5: fp = fopen("5.bmp", "wb"); break;
		case 4 : fp = fopen("4.bmp", "wb"); break;
		case 3 : fp = fopen("3.bmp", "wb"); break;
		case 2 : fp = fopen("2.bmp", "wb"); break;
		case 1 : fp = fopen("1.bmp", "wb"); break;
		default : fp = fopen(CAPTURE_FILE, "wb"); break;
	}
*/
    if(!fp) return -1;
 
    fwrite(&bfh, 8, 1, fp);
    fwrite(&bfh.bfReserved2, sizeof(bfh.bfReserved2), 1, fp);
    fwrite(&bfh.bfOffBits, sizeof(bfh.bfOffBits), 1, fp);
    fwrite(&bih, sizeof(BMPINFOHEADER_T), 1, fp);
    fwrite(pdata, size, 1, fp);
    fclose(fp);
    return 1;
}


//////////////////////////////////////////////////////////////////////////
//         Process BMP24 via OpenCV
//////////////////////////////////////////////////////////////////////////

#include <opencv/cv.h>
#include <opencv/highgui.h> 
using namespace cv; 
using namespace std;

vector<int> process_lp () {
    Mat img = imread(CAPTURE_FILE);
    if(img.empty())    {
        printf("Could not load image file: %s\n", CAPTURE_FILE);
        exit(0);
    }
    vector<int> data = plr(img);
    int time_sec;
    t_finish = clock();
    time_sec = (t_finish - t_start)/CLOCKS_PER_SEC;
    data.push_back(time_sec);
    img.release();
    if(data[0] != 0) {
	if (flag) {
        		flag = false;
  		  }
   		 else {
			data[0] += 0x30;
			data[1] += 0x30;
			flag = true;
   		 }
    }
    else {
        data[0] = 0xff;
	data[1] = 0xff;
    }
    return (data);
}


//////////////////////////////////////////////////////////////////////////
//         Send messages to FPGA
//////////////////////////////////////////////////////////////////////////

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

int send_message_to_fpga (vector<int> mes) {
    void *virtual_base;
    int fd;
    void *h2p_time_sec_addr;
    void *h2p_lp_l_addr;
    void *h2p_lp_r_addr;

    if((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)    {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return -1;
    }

    virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
    if(virtual_base == MAP_FAILED) {
        printf("ERROR: mmap() failed...\n");
        close(fd);
        return -1;
    }
    
    h2p_time_sec_addr = virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + TIME_SEC_BASE ) & (unsigned long)HW_REGS_MASK);
    h2p_lp_l_addr = virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + LP_L_BASE ) & (unsigned long)HW_REGS_MASK);
    h2p_lp_r_addr = virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + LP_R_BASE ) & (unsigned long)HW_REGS_MASK);
    *(uint32_t *)h2p_lp_l_addr = mes[0]; 
    *(uint32_t *)h2p_lp_r_addr = mes[1]; 
    *(uint32_t *)h2p_time_sec_addr = mes[2]; 
    printf("time : %d\n", mes[2]);
    printf("lp_l : %x\n", mes[0]);
    printf("lp_r : %x\n", mes[1]);

    if(munmap(virtual_base, HW_REGS_SPAN) != 0) {
        printf("ERROR: munmap() failed...\n");
        close(fd);
        return -1;
    }

    close(fd);
    if (mes[0] != 0xff)  return 1;
    else                                return 0;
}



//////////////////////////////////////////////////////////////////////////
//         Main Function
//////////////////////////////////////////////////////////////////////////
typedef struct VideoBuffer {
    void   *start;
    size_t  length;
} VideoBuffer;

int main() {
    int i, ret;
    unsigned char prgb[VIDEO_WIDTH * VIDEO_HEIGHT * 3];

    //open uvc camera
    int fd = open(CAMERA_DEVICE, O_RDWR, 0);
    if(fd < 0)  {
        printf("Open %s failed\n", CAMERA_DEVICE);
        return fd;
    }

    //get driver infomation
    struct v4l2_capability cap;
    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    if(ret < 0) {
        printf("VIDIOC_QUERYCAP failed (%d)\n", ret);
        return ret;
    }

    //Print capability infomations
    printf("Capability Informations:\n");
    printf(" driver: %s\n", cap.driver);
    printf(" card: %s\n", cap.card);
    printf(" bus_info: %s\n", cap.bus_info);
    printf(" version: %08X\n", cap.version);
    printf(" capabilities: %08X\n", cap.capabilities);

    //show all supported formats
    struct v4l2_fmtdesc fmtdesc;
    fmtdesc.index = 0; //form number
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; //frame type  
    while(ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1)   {  
        //if(fmtdesc.pixelformat && fmt.fmt.pix.pixelformat){
        printf("VIDIOC_ENUM_FMT success.->fmt.fmt.pix.pixelformat : %s\n", fmtdesc.description);
        //}
        fmtdesc.index++;
    }
    
    //show all supported frame sizes
    struct v4l2_fmtdesc fmt_1;
    struct v4l2_frmsizeenum frmsize;
    struct v4l2_frmivalenum frmival;
    fmt_1.index = 0;
    fmt_1.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while(ioctl(fd, VIDIOC_ENUM_FMT, &fmt_1) >= 0)  {
        char fmtstr0[8];
        memset(fmtstr0, 0, 8);
        memcpy(fmtstr0, &fmt_1.pixelformat, 4);
        printf("pixelformat: %s\n", fmtstr0);
        frmsize.pixel_format = fmt_1.pixelformat;
        frmsize.index = 0;
        while(ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0) {
            if(frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE){
               printf("line:%d %dx%d\n",__LINE__, frmsize.discrete.width, frmsize.discrete.height);
            }
            else if(frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE){
               printf("line:%d %dx%d\n",__LINE__, frmsize.discrete.width, frmsize.discrete.height);
            }
            frmsize.index++;
        }
        fmt_1.index++;
    }

    //set video format
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = VIDEO_WIDTH;
    fmt.fmt.pix.height      = VIDEO_HEIGHT;
    fmt.fmt.pix.pixelformat = VIDEO_FORMAT;
    //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
    ret = ioctl(fd, VIDIOC_S_FMT, &fmt);
    if(ret < 0) {
        printf("VIDIOC_S_FMT failed (%d)\n", ret);
        return ret;
    }
    //get video format
    ret = ioctl(fd, VIDIOC_G_FMT, &fmt);
    if(ret < 0) {
        printf("VIDIOC_G_FMT failed (%d)\n", ret);
        return ret;
    }
    //Print Stream Format
    printf("Stream Format Informations:\n");
    printf(" type: %d\n", fmt.type);
    printf(" width: %d\n", fmt.fmt.pix.width);
    printf(" height: %d\n", fmt.fmt.pix.height);
    char fmtstr[8];
    memset(fmtstr, 0, 8);
    memcpy(fmtstr, &fmt.fmt.pix.pixelformat, 4);
    printf(" pixelformat: %s\n", fmtstr);
    printf(" field: %d\n", fmt.fmt.pix.field);
    printf(" bytesperline: %d\n", fmt.fmt.pix.bytesperline);
    printf(" sizeimage: %d\n", fmt.fmt.pix.sizeimage);
    printf(" colorspace: %d\n", fmt.fmt.pix.colorspace);
    printf(" priv: %d\n", fmt.fmt.pix.priv);
    printf(" raw_date: %s\n", fmt.fmt.raw_data);

    //request for frame buffer
    struct v4l2_requestbuffers reqbuf;    
    reqbuf.count = BUFFER_COUNT;  // frame number
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;  // memory map mode
    
    ret = ioctl(fd , VIDIOC_REQBUFS, &reqbuf);
    if(ret < 0) {
        printf("VIDIOC_REQBUFS failed (%d)\n", ret);
        return ret;
    }

    //map the frame buffer from the kernel space to user space
    //allocate frame buffer in user space
    VideoBuffer* framebuf = (VideoBuffer*)calloc(reqbuf.count, sizeof(VideoBuffer));
    struct v4l2_buffer buf;
    //MMAP 
    for(i = 0; i < reqbuf.count; i++)   {
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        ret = ioctl(fd , VIDIOC_QUERYBUF, &buf);
        if(ret < 0) {
            printf("VIDIOC_QUERYBUF (%d) failed (%d)\n", i, ret);
            return ret;
        }

        //mmap buffer
        framebuf[i].length = buf.length;
        framebuf[i].start = (char*)mmap(0, buf.length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if(framebuf[i].start == MAP_FAILED) {
            printf("mmap (%d) failed: %s\n", i, strerror(errno));
            return -1;
        }
    
        //push frame buffer to queue
        ret = ioctl(fd , VIDIOC_QBUF, &buf);
        if(ret < 0) {
            printf("VIDIOC_QBUF (%d) failed (%d)\n", i, ret);
            return ret;
        }

        printf("Frame buffer %d: address=0x%x, length=%d\n", i, (unsigned int)framebuf[i].start, framebuf[i].length);
    }

    //start video capture
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd, VIDIOC_STREAMON, &type);
    if(ret < 0) {
        printf("VIDIOC_STREAMON failed (%d)\n", ret);
        return ret;
    }
    t_start = clock();
    t_finish = t_start;
    flag = false;
    //process video
    while(1){
        //init select listening
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 500000;
        ret = select(fd+1, &fds, NULL, NULL, &tv);
        if (ret == -1)  {
            printf("error : listen failed\n");
        } 
        else if (ret == 0)  {
            printf("listen time out!\n");
        } 
        else    {
            //Get frame
            ret = ioctl(fd, VIDIOC_DQBUF, &buf);
            if (ret < 0) {
                printf("VIDIOC_DQBUF failed (%d)\n", ret);
                return ret;
            }
            
            //Process the frame
            uyvy422_to_rgb((unsigned char*)framebuf[buf.index].start, prgb);
            ret = rgb_to_bmp(prgb);
            if (ret < 0) {
                printf("rgb_to_bmp failed\n");
                return ret;
            }
            printf("Capture one frame saved in %s\n", CAPTURE_FILE);
            ret = send_message_to_fpga(process_lp());
            if (ret < 0) {
            printf("send_message_to_fpga failed\n");
             return ret;
           }
           if (ret == 1)  sleep(8);

            //Re-queen buffer
            ret = ioctl(fd, VIDIOC_QBUF, &buf);
            if (ret < 0) {
                printf("VIDIOC_QBUF failed (%d)\n", ret);
                return ret;
            }
        }
    }

    //stop video capture
    ret = ioctl(fd, VIDIOC_STREAMOFF, &type);
    if (ret < 0) {
        printf("VIDIOC_STREAMON failed (%d)\n", ret);
        return ret;
    }
    // Release the resource
    for (i = 0; i < 4; i++) 
        munmap(framebuf[i].start, framebuf[i].length);
    close(fd);
    printf("Camera closed.\n");
	return 0;
}
