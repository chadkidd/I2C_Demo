/*
 * Copyright (C) Advanced Input Systems 2018.
 *
 * Author: Chad Kidd
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the PG_ORGANIZATION nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY	THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS-IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#define UNIX_PATH_MAX   108

int main(void)
{
 struct sockaddr_un address;
 int  socket_fd, nbytes;
 char buffer[16];
 char input_str[100];

int16_t iThe, iRho = 0;
int8_t calStrength = 0;
int8_t status = 0;
int16_t magMeasCount = 0;
int16_t magInterference = 0;

 socket_fd = socket(PF_LOCAL, SOCK_STREAM, 0);
 if(socket_fd < 0)
 {
	  printf("socket() failed\n");
	  return 1;
 }

 /* start with a clean address structure */
 memset(&address, 0, sizeof(struct sockaddr_un));

 address.sun_family = AF_LOCAL;
 snprintf(address.sun_path, UNIX_PATH_MAX, "/tmp/ecompass_socket");

 if(connect(socket_fd,
            (struct sockaddr *) &address,
            sizeof(struct sockaddr_un)) != 0)
 {
	  printf("connect error: %s\n", strerror(errno));
	  return 1;
 }

 while(1)
 {
	 printf("CMD to Send:");
	 scanf("%s", input_str);
	 write(socket_fd, input_str, 16);
	 //nbytes = snprintf(buffer, 16, "REQ#");
	 //write(socket_fd, buffer, 16);

 	 nbytes = read(socket_fd, buffer, 16);
 	 status = buffer[2];
	 iThe = (buffer[6] << 8) | buffer[5];
	 iRho = (buffer[4] << 8) | buffer[3];
	 calStrength = buffer[7];
	 magMeasCount = (buffer[9] << 8) | buffer[8];
	 magInterference = buffer[10];

 	 printf("Head,Ang,Strgth,Cnt,Intf,Status: %d, %d, %d, %d, %d, %d\n",
 			 iRho, iThe, calStrength, magMeasCount, magInterference, status);
 	 usleep(100000);
 }

 close(socket_fd);

 return 0;
}
