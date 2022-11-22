/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Universität Stuttgart
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

struct frame_header
{
    size_t length;
    uint32_t sec;
    uint32_t nsec;
    unsigned int pixelformat;
    int width;
    int height;
};

#define MAXSIZE (10000*10000*3)

int main(int argc, char** argv) {

    if (argc!=2 || strncmp(argv[argc-1],"--help",6)==0 || strncmp(argv[argc-1],"-h",2)==0) {
        printf("Usage: %s <streamfile>\n",argv[0]);
        printf("Reads the stream from <streamfile> and pipes it to STDOUT\n");
        printf("Information is written to STDERR\n");
        exit(0);
    }
    int fd=open(argv[1],O_RDONLY);
    if (fd==-1) {
        fprintf(stderr,"Error opening %s\n",argv[1]);
        exit(1);
    }

    struct frame_header header;
    int i=0;
    char format[5]="XXXX";
    char * buffer=malloc(MAXSIZE);
    if (!buffer) {
        fprintf(stderr,"Buffer MALLOC error!\n");
        exit(1);
    }
    while (read(fd,&header,sizeof(header))==sizeof(header)) {
        format[0]=(char)(header.pixelformat & 0xff);
        format[1]=(char)((header.pixelformat>>8) & 0xff);
        format[2]=(char)((header.pixelformat>>(2*8)) & 0xff);
        format[3]=(char)((header.pixelformat>>(3*8)) & 0xff);
        fprintf(stderr,"Frame %i: %ix%i at %i.%i fmt: %s, %lu bytes\n",i++,header.width,header.height,header.sec,header.nsec,format,header.length);
        if (header.length>MAXSIZE) {
            fprintf(stderr,"Error: Frame too large for buffer, maximum: %u\n",MAXSIZE);
            exit(1);
        }
        if(read(fd,buffer,header.length)!=header.length) {
            fprintf(stderr,"Read Error\n");
            exit(1);
        }
        if(write(1,buffer,header.length)!=header.length) {
            fprintf(stderr,"Write Error\n");
            exit(1);
        }
    }
    exit(0);
}
