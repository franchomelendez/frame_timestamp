#include <libavutil/imgutils.h>
#include <libavformat/rtsp.h>
#include <stdio.h>
#include <sys/time.h>


double ntp_timestamp(AVFormatContext *pFormatCtx, uint32_t *last_rtcp_ts, double *base_time) {
	RTSPState* rtsp_state = (RTSPState*) pFormatCtx->priv_data;
	RTSPStream* rtsp_stream = rtsp_state->rtsp_streams[0];
	RTPDemuxContext* rtp_demux_context = (RTPDemuxContext*) rtsp_stream->transport_priv;

	
	// printf("timestamp:                %u\n", rtp_demux_context->timestamp);
	// printf("base_timestamp:           %u\n", rtp_demux_context->base_timestamp);
	// // printf("cur_timestamp:            %u\n", rtp_demux_context->cur_timestamp);
	// printf("last_rtcp_ntp_time:       %lu\n", rtp_demux_context->last_rtcp_ntp_time);
	// printf("last_rtcp_reception_time: %ld\n", rtp_demux_context->last_rtcp_reception_time);
	// printf("first_rtcp_ntp_time:      %lu\n", rtp_demux_context->first_rtcp_ntp_time);
	// printf("last_rtcp_timestamp:      %u\n", rtp_demux_context->last_rtcp_timestamp);

	// printf("diff: %d\n",(rtp_demux_context->timestamp - rtp_demux_context->base_timestamp));
	// printf("====================================\n");

	uint32_t new_rtcp_ts = rtp_demux_context->last_rtcp_timestamp;
	uint64_t last_ntp_time = 0;

	if(new_rtcp_ts != *last_rtcp_ts){
		*last_rtcp_ts=new_rtcp_ts;
		last_ntp_time = rtp_demux_context->last_rtcp_ntp_time;
		uint32_t seconds = ((last_ntp_time >> 32) & 0xffffffff)-2208988800;
		uint32_t fraction  = (last_ntp_time & 0xffffffff);
		double useconds = ((double) fraction / 0xffffffff);
		*base_time = seconds+useconds;
	}

	int32_t d_ts = rtp_demux_context->timestamp - *last_rtcp_ts;
	return *base_time + d_ts/90000.0;
}

double clock_timestamp()
{
	struct timeval tv;
    gettimeofday(&tv, NULL);
	double ts = (double)tv.tv_sec;
	double ts_usec =+ (double)tv.tv_usec/1000000.0;
	ts = ts + ts_usec;
    printf("Seconds since Jan. 1, 1970: %lf\n", ts);
    return ts;
}


int main(int argc, char *argv[]) {
	// av_log_set_level(AV_LOG_DEBUG);
	// av_log_set_level(0);

	AVCodecParameters * origin_par = NULL;
	AVCodec           * pCodec = NULL;
	AVFormatContext   * pFormatCtx = NULL;
	AVCodecContext    * pCodecCtx = NULL;
	AVFrame           * pFrame = NULL;
    // Image_FFMPEG picture;
    // AVFrame rgb_frame;

	AVPacket          packet;
	int               videoStream = -1;
	// uint32_t		frame_size = 640*368*1.5;

	uint8_t network_mode = 1;
	int result;
	char* rtsp_source = "rtsp://admin:@10.130.11.145:554/h264Preview_01_sub";
//  char* rtsp_source = "rtsp://admin:AIRS2022HIK@10.130.11.64:554/Streaming/channels/101/0:0";

    av_register_all();
	avformat_network_init();

	av_init_packet(&packet);

	AVDictionary* opts = NULL;
	av_dict_set(&opts, "stimeout", "5000000", 0);

	if (network_mode == 0) {
		av_log(NULL, AV_LOG_DEBUG, "Opening UDP stream\n");
		result = avformat_open_input(&pFormatCtx, rtsp_source, NULL, &opts);
	} else {
		av_dict_set(&opts, "rtsp_transport", "tcp", 0);

		av_log(NULL, AV_LOG_DEBUG, "Opening TCP stream\n");
		result = avformat_open_input(&pFormatCtx, rtsp_source, NULL, &opts);
	}

	if (result < 0) {
		av_log(NULL, AV_LOG_ERROR, "Couldn't open stream\n");
		return 0;
	}

	av_log(NULL, AV_LOG_DEBUG, "Opened stream\n");

	result = avformat_find_stream_info(pFormatCtx, NULL);
	if (result < 0) {
		av_log(NULL, AV_LOG_ERROR, "Couldn't find stream information\n");
		return 0;
	}

	videoStream = av_find_best_stream(pFormatCtx, AVMEDIA_TYPE_VIDEO, -1, -1, &pCodec, 0);
	if (videoStream == -1) {
		av_log(NULL, AV_LOG_ERROR, "Couldn't find video stream\n");
		return 0;
	}

	origin_par = pFormatCtx->streams[videoStream]->codecpar;
	//pCodec = avcodec_find_decoder(origin_par->codec_id);
	//pCodec = avcodec_find_decoder_by_name("h264_cuvid");

	if (pCodec == NULL) {
		av_log(NULL, AV_LOG_ERROR, "Unsupported codec\n");
		return 0;
	}

	pCodecCtx = avcodec_alloc_context3(pCodec);

	if (pCodecCtx == NULL) {
		av_log(NULL, AV_LOG_ERROR, "Couldn't allocate codec context\n");
		return 0;
	}

	result = avcodec_parameters_to_context(pCodecCtx, origin_par);
	if (result) {
		av_log(NULL, AV_LOG_ERROR, "Couldn't copy decoder context\n");
		return 0;
	}

	result = avcodec_open2(pCodecCtx, pCodec, NULL);
	if (result < 0) {
		av_log(NULL, AV_LOG_ERROR, "Couldn't open decoder\n");
		return 0;
	}

	pFrame = av_frame_alloc();
	if (pFrame == NULL) {
		av_log(NULL, AV_LOG_ERROR, "Couldn't allocate frame\n");
		return 0;
	}

    av_dump_format(pFormatCtx, 0, rtsp_source, 0);
	//int byte_buffer_size = av_image_get_buffer_size(pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height, 16);
	//byte_buffer_size = byte_buffer_size < frame_size ? byte_buffer_size : frame_size;

	int number_of_written_bytes;
	int got_frame;
	uint32_t last_rtcp_ts = 0;
	double base_time = 0;

	uint8_t* frame_data = NULL;

	FILE *output_file = NULL;

	// bool valid = false;

	double last_ts = 0;
	int64_t last_pts;

	while(1)
	{
		av_packet_unref(&packet);
		while(av_read_frame(pFormatCtx, &packet) >= 0) 
		{
			// av_packet_unref(&packet);
			// read next packet from the stream
			
			// if the packet is not from the video stream don't do anything and get next packet
			if (packet.stream_index != videoStream) {
				av_packet_unref(&packet);
				continue;
			}

			// decode the video frame
			if (avcodec_decode_video2(pCodecCtx, pFrame, &got_frame, &packet) < 0) { 
			// if (decode(&got_frame, pFrame, pCodecCtx, &packet) < 0) {
				av_log(NULL, AV_LOG_ERROR, "Decoding error\n");
				break;
			}

			if(got_frame) 
			{
                double clock_ts = clock_timestamp();
				double ts = ntp_timestamp(pFormatCtx, &last_rtcp_ts, &base_time);
				printf("Timestamp %lf\n", ts);
				printf("%lf\n", ts-last_ts);
				printf("Delay %lf\n", clock_ts-ts);
				last_ts = ts;

				printf("PTS %ld\n", packet.pts);
				printf("%ld\n", packet.pts-last_pts);
				last_pts = packet.pts;
				// frame
				// frame_data = av_malloc(byte_buffer_size);

				// if (!frame_data) {
				// 	av_log(NULL, AV_LOG_ERROR, "Couldn't allocate buffer\n");
				// 	break;
				// }

				// number_of_written_bytes = av_image_copy_to_buffer(
				// 	frame_data, byte_buffer_size,
				// 	(const uint8_t* const *) pFrame->data,
				// 	(const int*) pFrame->linesize,
				// 	pCodecCtx->pix_fmt,
				// 	pCodecCtx->width, pCodecCtx->height, 1);

				// if (number_of_written_bytes < 0) {
				// 	av_log(NULL, AV_LOG_ERROR, "Couldn't copy to buffer\n");
				// 	break;
				// }

				// //write YUV frames to files
				// /*char file_name_buf[30];
				// snprintf(file_name_buf, 30, "output/ts_%f_yuv", ts);

				// output_file = fopen(file_name_buf, "w+");

				// if (fwrite(frame_data, 1, byte_buffer_size, output_file) < 0) {
				// 	fprintf(stderr, "Failed to dump raw data.\n");
				// } else {
				// 	fclose(output_file);
				// }*/

				// free(frame_data);

				// av_packet_unref(&packet);
				// av_init_packet(&packet);

				// valid = true;

			}
		}
	} 	
	// while (av_read_frame(pFormatCtx, &packet) >= 0) {
		
	// 	// if (decode(&got_frame, pFrame, pCodecCtx, &packet) < 0) {
	// 	if (avcodec_decode_video2(pCodecCtx, pFrame, &got_frame, &packet)){
	// 		av_log(NULL, AV_LOG_ERROR, "Decoding error\n");
	// 		continue;
	// 	}

	// 	if (got_frame) {
	// 		double ts = ntp_timestamp(pFormatCtx, &last_rtcp_ts, &base_time);

	// 		// frame
	// 		frame_data = av_malloc(byte_buffer_size);

	// 		if (!frame_data) {
	// 			av_log(NULL, AV_LOG_ERROR, "Couldn't allocate buffer\n");
	// 			break;
	// 		}

	// 		number_of_written_bytes = av_image_copy_to_buffer(
	// 			frame_data, byte_buffer_size,
	// 			(const uint8_t* const *) pFrame->data,
	// 			(const int*) pFrame->linesize,
	// 			pCodecCtx->pix_fmt,
	// 			pCodecCtx->width, pCodecCtx->height, 1);

	// 		if (number_of_written_bytes < 0) {
	// 			av_log(NULL, AV_LOG_ERROR, "Couldn't copy to buffer\n");
	// 			break;
	// 		}

	// 		//write YUV frames to files
	// 		/*char file_name_buf[30];
	// 		snprintf(file_name_buf, 30, "output/ts_%f_yuv", ts);

	// 		output_file = fopen(file_name_buf, "w+");

	// 		if (fwrite(frame_data, 1, byte_buffer_size, output_file) < 0) {
	// 			fprintf(stderr, "Failed to dump raw data.\n");
    //   } else {
	// 			fclose(output_file);
	// 		}*/

	// 		free(frame_data);

	// 		av_packet_unref(&packet);
	// 		av_init_packet(&packet);
	// 	}
	// }
	return 0;
}
