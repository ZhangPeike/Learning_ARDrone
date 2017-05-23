#pragma once

#include <opencv2/opencv.hpp>

#ifndef WIN32
//to fix the ffmepg compiler error in ubuntu
#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif
#endif

extern "C" {
        #include <libavcodec/avcodec.h>
        #include <libavformat/avformat.h>
        #include <libswscale/swscale.h>
}

using namespace std;
using namespace cv;

#define PAVE_INT32BE_SIGNATURE (0x50615645)
#define PAVE_INT32LE_SIGNATURE (0x45566150)
#define PAVE_CHECK(x) ( (*((uint32_t*)(x)))==PAVE_INT32LE_SIGNATURE )

typedef struct {
	/*00*/ uint8_t  signature[4];
	/*04*/ uint8_t  version;
	/*05*/ uint8_t  video_codec;
	/*06*/ uint16_t header_size;
	/*08*/ uint32_t payload_size;             /* Amount of data following this PaVE */
	/*12*/ uint16_t encoded_stream_width;     /* ex: 640 */
	/*14*/ uint16_t encoded_stream_height;    /* ex: 368 */
	/*16*/ uint16_t display_width;            /* ex: 640 */
	/*18*/ uint16_t display_height;           /* ex: 360 */
	/*20*/ uint32_t frame_number;             /* frame position inside the current stream */
	/*24*/ uint32_t timestamp;                /* in milliseconds */
	/*28*/ uint8_t  total_chuncks;            /* number of UDP packets containing the current decodable payload */
	/*29*/ uint8_t  chunck_index ;            /* position of the packet - first chunk is #0 */
	/*30*/ uint8_t  frame_type;               /* I-frame, P-frame */
	/*31*/ uint8_t  control;                  /* Special commands like end-of-stream or advertised frames */
	/*32*/ uint32_t stream_byte_position_lw;  /* Byte position of the current payload in the encoded stream  - lower 32-bit word */
	/*36*/ uint32_t stream_byte_position_uw;  /* Byte position of the current payload in the encoded stream  - upper 32-bit word */
	/*40*/ uint16_t stream_id;                /* This ID indentifies packets that should be recorded together */
	/*42*/ uint8_t  total_slices;             /* number of slices composing the current frame */
	/*43*/ uint8_t  slice_index ;             /* position of the current slice in the frame */
	/*44*/ uint8_t  header1_size;             /* H.264 only : size of SPS inside payload - no SPS present if value is zero */
	/*45*/ uint8_t  header2_size;             /* H.264 only : size of PPS inside payload - no PPS present if value is zero */
	/*46*/ uint8_t  reserved2[2];             /* Padding to align on 48 bytes */
	/*48*/ uint32_t advertised_size;          /* Size of frames announced as advertised frames */
	/*52*/ uint8_t  reserved3[12];            /* Padding to align on 64 bytes */
} /**__attribute__ ((packed))**/ PaVE;


class ARVideo
{
public:
	int _w,_h;
	ARVideo(string path="http://192.168.1.1:5555");
	~ARVideo(void);

	bool queryFrame(IplImage ** image);
	bool queryLatestFrame(IplImage ** image);
	virtual void open();
	virtual void grabFrame(){/*do nothing*/}
	virtual void readCurFrameRGB(unsigned char* imgdata);
	virtual void readCurFrameGray(unsigned char* grayImgData);
	virtual void readCurFrame(unsigned char* rgbdata, unsigned char* graydata);
	virtual void skip(int nSkippedFrame){};
	virtual int getTotalFrame(){return 0;}
	void setCurrentPaVE();
	virtual uint32_t getTimeStamp(){return 0;}

	Size size;
    float fps;
	bool opened;
	int frames;
protected:
	virtual void close();

	AVFormatContext *pFormatCtx;
	AVCodecContext  *pCodecCtx;
	AVCodec         *pCodec;
	AVFrame         *pFrame;
	AVFrame         *pFrame_BGR24;
	uint8_t			*buffer;
    AVRational      frameRate;
    AVPicture       picture;
	AVPacket        packet;
	int				videoStream;
    IplImage        *frame;    
	struct SwsContext *pImgConvertCtx;
	char* ip_string;
public:
	// Frame header information
	PaVE pave;
};