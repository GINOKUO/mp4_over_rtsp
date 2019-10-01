#include "stdafx.h"
#include "mp4.h"
#include <malloc.h>
#include <string.h>


#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "rtp.h"

#define CLIENT_IP       "192.168.50.5"
#define CLIENT_PORT     8554
#define FPS             25


static int createUdpSocket()
{
    int fd;
    int on = 1;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(fd < 0)
        return -1;

    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on));

    return fd;
}

static int rtpSendH264Frame(int socket, char* ip, int16_t port,
                            struct RtpPacket* rtpPacket, uint8_t* frame, uint32_t frameSize)
{
    uint8_t naluType; // nalu第一?字?
    int sendBytes = 0;
    int ret;

    naluType = frame[0];

    if (frameSize <= RTP_MAX_PKT_SIZE) // nalu?度小于最大包?：?一NALU?元模式
    {
        /*
         *   0 1 2 3 4 5 6 7 8 9
         *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
         *  |F|NRI|  Type   | a single NAL unit ... |
         *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
         */
        memcpy(rtpPacket->payload, frame, frameSize);
        ret = rtpSendPacket(socket, ip, port, rtpPacket, frameSize);
        if(ret < 0)
            return -1;

        rtpPacket->rtpHeader.seq++;
        sendBytes += ret;
        if ((naluType & 0x1F) == 7 || (naluType & 0x1F) == 8) // 如果是SPS、PPS就不需要加??戳
            goto out;
    }
    else // nalu?度小于最大包?：分片模式
    {
        /*
         *  0                   1                   2
         *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3
         * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
         * | FU indicator  |   FU header   |   FU payload   ...  |
         * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
         */

        /*
         *     FU Indicator
         *    0 1 2 3 4 5 6 7
         *   +-+-+-+-+-+-+-+-+
         *   |F|NRI|  Type   |
         *   +---------------+
         */

        /*
         *      FU Header
         *    0 1 2 3 4 5 6 7
         *   +-+-+-+-+-+-+-+-+
         *   |S|E|R|  Type   |
         *   +---------------+
         */

        int pktNum = frameSize / RTP_MAX_PKT_SIZE;       // 有几?完整的包
        int remainPktSize = frameSize % RTP_MAX_PKT_SIZE; // 剩余不完整包的大小
        int i, pos = 1;

        /* ?送完整的包 */
        for (i = 0; i < pktNum; i++)
        {
            rtpPacket->payload[0] = (naluType & 0x60) | 28;
            rtpPacket->payload[1] = naluType & 0x1F;
            
            if (i == 0) //第一包?据
                rtpPacket->payload[1] |= 0x80; // start
            else if (remainPktSize == 0 && i == pktNum - 1) //最后一包?据
                rtpPacket->payload[1] |= 0x40; // end

            memcpy(rtpPacket->payload+2, frame+pos, RTP_MAX_PKT_SIZE);
            ret = rtpSendPacket(socket, ip, port, rtpPacket, RTP_MAX_PKT_SIZE+2);
            if(ret < 0)
                return -1;

            rtpPacket->rtpHeader.seq++;
            sendBytes += ret;
            pos += RTP_MAX_PKT_SIZE;
        }

        /* ?送剩余的?据 */
        if (remainPktSize > 0)
        {
            rtpPacket->payload[0] = (naluType & 0x60) | 28;
            rtpPacket->payload[1] = naluType & 0x1F;
            rtpPacket->payload[1] |= 0x40; //end

            memcpy(rtpPacket->payload+2, frame+pos, remainPktSize+2);
            ret = rtpSendPacket(socket, ip, port, rtpPacket, remainPktSize+2);
            if(ret < 0)
                return -1;

            rtpPacket->rtpHeader.seq++;
            sendBytes += ret;
        }
    }

out:

    return sendBytes;
}

int main(int argc, char* argv[])
{
   int sampleCount;
   mp4_box_t *root = NULL, *stsz = NULL, *stco = NULL, *stsc = NULL;
   stream_t* fd = NULL;

   unsigned long filesize = 0;
   BUFFER_t *buffer = NULL;
   FILE *file = fopen("test.mp4","rb");
   fseek(file,0L,SEEK_END);
   filesize = ftell(file);
   fseek(file,0L,SEEK_SET); 
   buffer = (BUFFER_t *)malloc(sizeof(BUFFER_t));
   buffer->begin_addr = (unsigned char *)malloc(filesize);
   buffer->buf = (unsigned char *)malloc(filesize);
   fread(buffer->begin_addr,filesize,1,file);
   memcpy(buffer->buf,buffer->begin_addr,filesize);
   (*buffer).offset = 0;
   (*buffer).filesize = filesize;
   fd = create_buffer_stream();
   if (buffer_open(fd, buffer) == 0)
      return -1;

   root = MP4_BoxGetRootFromBuffer(fd,filesize);
   stsz = MP4_BoxSearchBox(root,ATOM_stsz);
   printf("box is %c%c%c%c  type %x  sample_count %d\n"
		,stsz->i_type&0x000000ff
		,(stsz->i_type&0x0000ff00)>>8
		,(stsz->i_type&0x00ff0000)>>16
		,(stsz->i_type&0xff000000)>>24
		,stsz->i_type
		,stsz->data.p_stsz->sample_count);
		
   int stsz_sample_count = stsz->data.p_stsz->sample_count;
   int stsz_smaple_size[stsz_sample_count];
   for(sampleCount = 0; sampleCount < stsz_sample_count; sampleCount++)
   {
	   stsz_smaple_size[sampleCount] = stsz->data.p_stsz->entry_size[sampleCount];
	   //printf("No %d smaple size %d\n",sampleCount+1,stsz_smaple_size[sampleCount]);
   }
   stco = MP4_BoxSearchBox(root,ATOM_stco);
   printf("box is %c%c%c%c  type %x  sample_count %d\n"
		,stco->i_type&0x000000ff
		,(stco->i_type&0x0000ff00)>>8
		,(stco->i_type&0x00ff0000)>>16
		,(stco->i_type&0xff000000)>>24
		,stco->i_type
		,stco->data.p_stco->sample_size);
		
   int stco_entry_count = stco->data.p_stco->sample_size;
   int stco_chunk_offset[stco_entry_count];
   for(sampleCount = 0; sampleCount < stco_entry_count; sampleCount++) 
   {
	   stco_chunk_offset[sampleCount] = stco->data.p_stco->entry_size[sampleCount*2];
  	   //printf("No %d Chunk offset %d\n",sampleCount+1,stco_chunk_offset[sampleCount]);
   }
   	
   stsc = MP4_BoxSearchBox(root,ATOM_stsc);
   printf("box is %c%c%c%c  type %x  entry_count %d\n"
		,stsc->i_type&0x000000ff
		,(stsc->i_type&0x0000ff00)>>8
		,(stsc->i_type&0x00ff0000)>>16
		,(stsc->i_type&0xff000000)>>24
		,stsc->i_type
		,stsc->data.p_stsc->entry_count);
	
	int stsc_entry_count = stsc->data.p_stsc->entry_count;
	int stsc_first_chunk[stsc_entry_count];
	int stsc_samples_per_chunk[stsc_entry_count];
	for(sampleCount = 0; sampleCount < stsc_entry_count; sampleCount++) 
    {
	   stsc_first_chunk[sampleCount] = stsc->data.p_stsc->first_chunk[sampleCount];
	   stsc_samples_per_chunk[sampleCount] = stsc->data.p_stsc->samples_per_chunk[sampleCount];
  	   //printf("No %d Chunk offset %d\n",sampleCount+1,stco_chunk_offset[sampleCount]);
    }
   	
   MP4_BoxFreeFromBuffer( root );
   buffer_close(fd);
   destory_buffer_stream(fd);
      
   ////////////////
 
   int buff_tmp[4] = {0, 0, 0, 0};
   int avcC_type[4] = {0x43, 0x63, 0x76, 0x61};
   int avcC_start_offset = 0;
   int sps_len_offset[2] = {0, 0};
   int sps_len = 0;
   int sps_start_offset = 0;
   int pps_len_offset[2] = {0, 0};
   int pps_len = 0;
   int pps_start_offset = 0;
   int offset = 0;
   int startCodeNum = 4;
   int stsz_sample_count_num = 0;
   int sco_count = 0;
   int chunk = 0;
   int frame_count = 0;
   int i ;
   uint8_t *frame_buff = (uint8_t*)malloc(300000);
   
   
   
   
   
   int socket;
   struct RtpPacket* rtpPacket;
   socket = createUdpSocket();
   rtpPacket = (struct RtpPacket*)malloc(500000);
   rtpHeaderInit(rtpPacket, 0, 0, 0, RTP_VESION, RTP_PAYLOAD_TYPE_H264, 0,
                    0, 0, 0x88923423);

   fd = create_file_stream();
   if (stream_open(fd, "test.mp4", MODE_READ) == 0)
      return -1;
  
   while(1)
   {
	  if(offset < stco_chunk_offset[sco_count])
	  {
		  if(offset == sps_start_offset)
		  {
			  stream_read(fd, frame_buff, sps_len);
			  /*
			  for(i  = 0; i < sps_len; i++) 
			  {
				printf("%x ",frame_buff[i]);
			  }
			  */
			  rtpSendH264Frame(socket, CLIENT_IP, CLIENT_PORT,
                            rtpPacket, frame_buff, sps_len);
			  rtpPacket->rtpHeader.timestamp += 90000/FPS;
			  usleep(1000*1000/FPS);
			  offset+=sps_len;
		  }
		  else if(offset == pps_start_offset)
		  {
			  stream_read(fd, frame_buff, pps_len);
			  /*
			  for(i  = 0; i < pps_len; i++) 
			  {
				printf("%x ",frame_buff[i]);
			  }*/
			  rtpSendH264Frame(socket, CLIENT_IP, CLIENT_PORT,
                            rtpPacket, frame_buff, pps_len);
			  rtpPacket->rtpHeader.timestamp += 90000/FPS;
			  usleep(1000*1000/FPS);
			  offset+=pps_len;
		  }
		  
		  stream_read(fd, frame_buff, 1);
		  buff_tmp[3] = buff_tmp[2];
		  buff_tmp[2] = buff_tmp[1];
		  buff_tmp[1] = buff_tmp[0];
		  buff_tmp[0] = frame_buff[0];
		  if(buff_tmp[0] == avcC_type[0] && buff_tmp[1] == avcC_type[1] && buff_tmp[2] == avcC_type[2] && buff_tmp[3] == avcC_type[3])
		  {
			  avcC_start_offset = offset;
			  sps_len_offset[0] = avcC_start_offset + 7;
			  sps_len_offset[1] = sps_len_offset[0] + 1;
			  sps_start_offset = sps_len_offset[1] + 1;
		  }
		  // get sps offset
		  if(offset == sps_len_offset[0] || offset == sps_len_offset[1])
		  {
			  if(offset == sps_len_offset[0])
			  {
				  sps_len = frame_buff[0] << 8;
			  } 
			  else 
			  {
				  sps_len += frame_buff[0];
				  pps_len_offset[0] = sps_start_offset + sps_len + 1;
				  pps_len_offset[1] = pps_len_offset[0] + 1;
				  pps_start_offset = pps_len_offset[1] + 1;
			  }
		  }
		
		  // get pps offset
		  if(offset == pps_len_offset[0] || offset == pps_len_offset[1]) 
		  {
			  if(offset == pps_len_offset[0])
			  {
				  pps_len = frame_buff[0] << 8;
			  } 
			  else 
			  {
				  pps_len += frame_buff[0];
			  }
		  }
	  	  offset++;
	  }
	 
	  else
	  {
		 frame_count++;
		 stream_read(fd, frame_buff, stsz_smaple_size[stsz_sample_count_num]);
		 
		 offset += stsz_smaple_size[stsz_sample_count_num]; 
		 /*
         for(i  = 0; i < stsz_smaple_size[stsz_sample_count_num] - 4; i++)
			printf("%x ",frame_buff[i+4]);
		 */
		 rtpSendH264Frame(socket, CLIENT_IP, CLIENT_PORT,
                            rtpPacket, frame_buff+4, stsz_smaple_size[stsz_sample_count_num]-4);
		 rtpPacket->rtpHeader.timestamp += 90000/FPS;
	     usleep(1000*1000/FPS);
		 stsz_sample_count_num++;
		 chunk++;
		 if(sco_count == stsc_first_chunk[1] - 1) {
			if(chunk == stsc_samples_per_chunk[1] )
			{
				sco_count++;
				chunk = 0;
			}
		 } 
		 else
		 {
			if(chunk == stsc_samples_per_chunk[0] )
			{
				sco_count++;
				chunk = 0;
			}
		 }
	  }
	  
	  if (frame_count == stsz_sample_count )
         break;
	 

   }
   
   stream_close(fd);
   destory_file_stream(fd);

  
	return 0;
}

