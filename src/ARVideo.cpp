#include "ARVideo.h"

using namespace std;
using namespace cv;

ARVideo::ARVideo(string path)
{
	ip_string = new char [path.size() + 1];
	strcpy(ip_string,path.c_str());
	frames = 0;
}

ARVideo::~ARVideo(void)
{
	close();
}

void ARVideo::open(){
	// FFMPEG initialisation
	av_register_all();
	avcodec_register_all();
	avformat_network_init();

	// Open input stream
	pFormatCtx = NULL;
	if (avformat_open_input(&pFormatCtx, ip_string, NULL, NULL) != 0) 
		printf("Could not open the video file\nRetrying...\n");

	// Retrieve stream information
	avformat_find_stream_info(pFormatCtx, NULL);

	// Search for first video stream
	videoStream = -1;
	for (int i=0; i < pFormatCtx->nb_streams; i++)
		if (pFormatCtx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO) {
			videoStream = i;
			frameRate = pFormatCtx->streams[videoStream]->r_frame_rate;
			fps = float(frameRate.num) / float(frameRate.den);
			break;
		}

		if (videoStream == -1) {
			cerr << "Error: Did not find video stream" << endl;
			close();
			exit(-1);
		}

		// Save the codec context for this stream
		pCodecCtx = pFormatCtx->streams[videoStream]->codec;

		// Find the decoder for the video stream
		pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
		if (pCodec == NULL) {
			cerr << "Error: Unsupported codec" << endl;
			close();
			exit(-1);
		}

		// Open codec
		if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
			cerr << "Error: Unable to open codec" << endl;
			close();
			exit(-1);
		}

		// Allocate frame
		pFrame_BGR24 = avcodec_alloc_frame();

		if(pFrame_BGR24 == NULL) {
			fprintf(stderr, "Could not allocate pFrame_BGR24\n");
		}

		// Determine size of the buffer required
		size = Size(pCodecCtx->width, pCodecCtx->height);
		_w = pCodecCtx->width;
		_h = pCodecCtx->height;

		int numBytes = avpicture_get_size(PIX_FMT_RGB24, size.width, size.height);

		// Allocate buffer
		buffer = (uint8_t *)av_malloc(numBytes*sizeof(uint8_t));

		// Assign appropriate parts of buffer to image planes in pFrame.
		avpicture_fill(&picture, buffer, PIX_FMT_RGB24, size.width, size.height);

		pImgConvertCtx = sws_getContext(size.width, size.height, pCodecCtx->pix_fmt,
			size.width, size.height, PIX_FMT_BGR24,
			SWS_BICUBIC, NULL, NULL, NULL);

		if (pImgConvertCtx == NULL) {
			cerr << "Error: Could not initialise convertion context" << endl;
			close();
			exit(-1);
		}
		pFrame = avcodec_alloc_frame();
		frame = cvCreateImage(size,IPL_DEPTH_8U,3);
}

void ARVideo::readCurFrameRGB(unsigned char* imgdata ){
	// Check if there are any more frames
	if (av_read_frame(pFormatCtx, &packet) < 0) {
		fprintf(stderr, "Could not read frame!\n");
	}

	// Check if the packet belonged to the video stream
	if (packet.stream_index != videoStream) {
		printf("packet not belong to the videoStrea");
	}

	// Check if we have decoded the frame
	int frameFinished;
	if (avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, &packet) < 0) {
		cerr << "Error: Could not read frame" << endl;
	}

	// Scale to source
	sws_scale(pImgConvertCtx, pFrame->data, pFrame->linesize, 0, 
		size.height, picture.data, picture.linesize);

	// Convert to OpenCV
	cv::Mat videoFrame(_h, _w, CV_8UC3,imgdata);
	cv::Mat rawFrame(_h,_w, CV_8UC3, picture.data[0]);
	cv::cvtColor(rawFrame, videoFrame,CV_RGB2BGR);
	av_free_packet(&packet);
}


void ARVideo::readCurFrameGray(unsigned char* imgdata){
	// Check if there are any more frames
	if (av_read_frame(pFormatCtx, &packet) < 0) {
		fprintf(stderr, "Could not read frame!\n");
	}

	// Check if the packet belonged to the video stream
	if (packet.stream_index != videoStream) {
		printf("packet not belong to the videoStrea");
	}

	// Check if we have decoded the frame
	int frameFinished;
	if (avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, &packet) < 0) {
		cerr << "Error: Could not read frame" << endl;
	}

	// Scale to source
	sws_scale(pImgConvertCtx, pFrame->data, pFrame->linesize, 0, 
		size.height, picture.data, picture.linesize);

	// Convert to OpenCV
	cv::Mat videoFrame(_h, _w, CV_8UC1, imgdata);
	cv::Mat rawFrame(_h, _w, CV_8UC3, picture.data[0]);
	cv::cvtColor(rawFrame, videoFrame, CV_RGB2GRAY);

	av_free_packet(&packet);
}

void ARVideo::readCurFrame(unsigned char* rgbdata, unsigned char* graydata){
	readCurFrameRGB(rgbdata);
	readCurFrameGray(graydata);
}

bool ARVideo::queryFrame(IplImage ** image) {
	// Check if there are any more frames
	if (av_read_frame(pFormatCtx, &packet) < 0) {
		fprintf(stderr, "Could not read frame!\n");
		return false;
	}
	
	setCurrentPaVE();
	//printf("frame timestamp: %u\n", pave.timestamp);
	// Check if the packet belonged to the video stream
	if (packet.stream_index != videoStream) {
		printf("packet not belong to the videoStrea");
		return false;
	}

	// Check if we have decoded the frame
    int frameFinished;
	if (avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, &packet) < 0) {
		cerr << "Error: Could not read frame" << endl;
		return false;
	}
    
    //if (!frameFinished) {
    //    return false;
    //}

	// Scale to source
	sws_scale(pImgConvertCtx, pFrame->data, pFrame->linesize, 0, 
		size.height, picture.data, picture.linesize);

	// Convert to OpenCV
	memcpy (frame->imageData, picture.data[0], frame->imageSize);
    cvConvertScale(frame, *image, 1.0, 0.0);
    av_free_packet(&packet);
    
    return true;
}

bool ARVideo::queryLatestFrame(IplImage ** image) {
    bool result = true, ok = false;
    while (result) {
        result = queryFrame(image);
        ok = ok || result;
    }
    return ok;
}

void ARVideo::close() {
	if (pImgConvertCtx) sws_freeContext(pImgConvertCtx); // Free scaling context

    if (buffer) av_free(buffer);
	if (pFrame) av_free(pFrame); // Free frame

	if (pCodecCtx) avcodec_close(pCodecCtx); // Close codec
	if (&pFormatCtx) avformat_close_input(&pFormatCtx); // Close input stream
}

void ARVideo::setCurrentPaVE(){
	uint8_t *currPos = packet.data;
	for (int i = 0; i < packet.size; i++){
		if( PAVE_CHECK(currPos) ){
			memcpy((void*)(&pave), (uint8_t*)currPos, sizeof(pave));
			break;
		}
		currPos++;
	}
}