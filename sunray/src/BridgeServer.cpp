/*
  Server.h - Server class for Raspberry Pi
  Copyright (c) 2016 Hristo Gochkov  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include "BridgeServer.h"


#define MAXEVENTS 64

// Asynchronous socket server - accepting multiple clients concurrently,
// multiplexing the connections with epoll
// https://github.com/eliben/code-for-blog/blob/master/2017/async-socket-server/epoll-server.c


void *serverThreadFun(void *user_data)
{
  BridgeServer *server = (BridgeServer*)user_data;
  while (true)
    {
      server->run();
      //usleep(300);
    }
  return NULL;
}


int16_t BridgeServer::setSocketOption(int16_t option, char* value, size_t len)
{
  return setsockopt(sockfd, SOL_SOCKET, option, value, len);
}

int16_t BridgeServer::setTimeout(uint32_t seconds)
{
  struct timeval tv;
  tv.tv_sec = seconds;
  tv.tv_usec = 0;
  if(setSocketOption(SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval)) < 0)
    return -1;
  return setSocketOption(SO_SNDTIMEO, (char *)&tv, sizeof(struct timeval));
}

size_t BridgeServer::write(const uint8_t *data, size_t len)
{
  return 0;
}

void BridgeServer::stopAll()
{
  wxLogMessage("server stopAll");
}

void BridgeServer::run()
{
  struct sockaddr_in _client;
  int16_t cs = sizeof(struct sockaddr_in);
  int16_t sock = ::accept(sockfd, (struct sockaddr *)&_client, (socklen_t*)&cs);
  pthread_mutex_lock( &eventsMutex );
  clientSock = sock;
  pthread_mutex_unlock( &eventsMutex );
}


BridgeClient BridgeServer::available()
{
  if(!_listening)
    {
      wxLogMessage("available(): not listening");
      return BridgeClient();
    }

  pthread_mutex_lock( &eventsMutex );
  int16_t sock = clientSock;
  clientSock = -1;
  pthread_mutex_unlock( &eventsMutex );

  if (sock < 0)
    {
      return BridgeClient();  // no client
    }

  //wxLogMessage("server accepting");
  BridgeClient *client = new BridgeClient(this, sock);

  return *client;  // NOTE: THIS RETURNS A COPY!
}


void BridgeServer::begin()
{
  clients=0;
  clientSock=-1;
  wxLogMessage("server begin port %d", _port);
  if(_listening)
    return;
  struct sockaddr_in server;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  wxLogMessage("sockfd=%d", sockfd);
  if (sockfd < 0)
    {
      wxLogMessage("socket error");
      return;
    }
  // connection will close immediately after closing the program; and next restart will be able to bind again.
  // https://stackoverflow.com/questions/24194961/how-do-i-use-setsockoptso-reuseaddr
  linger lin;
  lin.l_onoff = 0;
  lin.l_linger = 0;
  setsockopt(sockfd, SOL_SOCKET, SO_LINGER, (const char *)&lin, sizeof(int16_t));

  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(_port);
  if(bind(sockfd, (struct sockaddr *)&server, sizeof(server)) < 0)
    {
      wxLogMessage("bind error port %d",_port);
      return;
    }
  if(listen(sockfd, _max_clients) < 0)
    {
      wxLogMessage("listen error");
      return;
    }

  _listening = true;
  wxLogMessage("server listening");
  pthread_create(&thread_id, NULL, serverThreadFun, (void*)this);
}

void BridgeServer::end()
{
  wxLogMessage("server end");
  stopAll();
  close(sockfd);
  sockfd = -1;
  _listening = false;
}
