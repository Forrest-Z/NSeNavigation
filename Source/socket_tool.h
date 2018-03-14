/*
 * socket_tool.h
 *
 *  Created on: Mar 12, 2018
 *      Author: pengjiawei
 */

#ifndef SOCKET_TOOL_H_
#define SOCKET_TOOL_H_

#include <netinet/in.h>
#include<arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <boost/archive/binary_oarchive.hpp>
#include <fstream>
#include <sstream>
#include <log_tool.h>

void SocketSend(const std::string& ip,const std::string& port,std::string& file_name){
	logInfo << "ip : port = "<<ip<<":"<<port<<" . file name = "<<file_name;
	const int BUFF_SIZE = 1024;
	char buff[BUFF_SIZE];
    int sock_id;
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(atoi(port.c_str()));

    inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr);

    //connect socket
    if ((sock_id = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {

        printf("Create socket failed %d\n", sock_id);

    }

    int reuseflag = 1;
    setsockopt(sock_id, SOL_SOCKET, SO_REUSEADDR, (const char*) &reuseflag,
               sizeof(int)); //set reused

    struct linger clsflag;
    clsflag.l_onoff = 1;
    clsflag.l_linger = 2;
    setsockopt(sock_id, SOL_SOCKET, SO_LINGER, (const struct linger*) &clsflag,
               sizeof(struct linger)); //set closed

    int i_ret = connect(sock_id, (struct sockaddr *) &serv_addr,
                        sizeof(serv_addr));

    printf("finish connect\n");
    if (i_ret == -1) {
        printf("Connect?socket?failed\n");
        close(sock_id);
    }
    clock_t start,end;
    start = clock();

    //read file
    FILE* file = fopen(file_name.c_str(),"r");
    memset(buff,0,sizeof(buff));
    size_t file_block_length = 0;
    size_t file_total_length = 0;
    while ((file_block_length = fread(buff,sizeof(char),BUFF_SIZE,file)) > 0){
    	file_total_length += file_block_length;
        if (send(sock_id, buff, file_block_length, 0) < 0){
            printf("send failed\n");
            break;
        }
        bzero(buff,sizeof(buff));
    }
    fclose(file);
    end = clock();
    printf("file total length = %d\n",file_total_length);
    cout<<"execute time = "<<end-start<<endl;
    close(sock_id);
}




#endif /* SOCKET_TOOL_H_ */
