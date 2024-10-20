/*
  Client.h - Client class for Raspberry Pi
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
#include "BridgeClient.h"
#include "BridgeServer.h"


int16_t sock_connect(int16_t fd, struct sockaddr *addr, size_t len)
{
  return connect(fd, addr, len);
}

/*
int16_t sock_write(int16_t fd, void *data, size_t len){
  return write(fd, data, len);
}
*/

int16_t sock_read(int16_t fd, void *data, size_t len)
{
  return read(fd, data, len);
}

BridgeClient::~BridgeClient()
{
  //if (sockfd < 0) return;
  //wxLogMessage("discard client fd=%d\n", sockfd);
  //stop();
  //if(sockfd) close(sockfd);
}

int16_t BridgeClient::connect(IPAddress ip, uint16_t port)
{
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      wxLogMessage("client connect error - no socket");
      return 0;
    }
  uint32_t ip_addr = ip;
  struct sockaddr_in serveraddr;
  bzero((char *) &serveraddr, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  bcopy((const void *)(&ip_addr), (void *)&serveraddr.sin_addr.s_addr, 4);
  serveraddr.sin_port = htons(port);
  setTimeout(500 * 1000); // 500ms timeout
  if (sock_connect(sockfd, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) < 0)
    {
      wxLogMessage("client connect error");
      return 0;
    }
  wxLogMessage("client connected fd=%d\n", sockfd);
  _connected = true;
  return 1;
}

int16_t BridgeClient::connect(const char *host, uint16_t port)
{
  // cache server resolution
  if (server == NULL)
    {
      server = gethostbyname(host);
      if (server == NULL)
        {
          wxLogMessage("client connect error - no server");
          return 0;
        }
    }
  return connect(IPAddress((const uint8_t *)(server->h_addr)), port);
}

int16_t BridgeClient::setSocketOption(int16_t option, char* value, size_t len)
{
  return setsockopt(sockfd, SOL_SOCKET, option, value, len);
}

int16_t BridgeClient::setTimeout(uint32_t useconds)
{
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = useconds;
  if(setSocketOption(SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval)) < 0)
    {
      wxLogMessage("client set timeout error");
      return -1;
    }
  return setSocketOption(SO_SNDTIMEO, (char *)&tv, sizeof(struct timeval));
}

int16_t BridgeClient::setOption(int16_t option, int16_t *value)
{
  return setsockopt(sockfd, IPPROTO_TCP, option, (char *)value, sizeof(int16_t));
}

int16_t BridgeClient::getOption(int16_t option, int16_t *value)
{
  socklen_t size = sizeof(int16_t);
  return getsockopt(sockfd, IPPROTO_TCP, option, (char *)value, &size);
}

int16_t BridgeClient::setNoDelay(bool nodelay)
{
  int16_t flag = nodelay;
  return setOption(TCP_NODELAY, &flag);
}

bool BridgeClient::getNoDelay()
{
  int16_t flag = 0;
  getOption(TCP_NODELAY, &flag);
  return flag;
}

size_t BridgeClient::write(uint8_t data)
{
  return write(&data, 1);
}

size_t BridgeClient::write(const uint8_t *buf, size_t size)
{
  if(!_connected)
    {
      wxLogMessage("client write error - not connected");
      return 0;
    }
  //wxLogMessage("client write fd %d size %d\n", sockfd, size);
  int16_t res = send(sockfd, (void*)buf, size, MSG_DONTWAIT | MSG_NOSIGNAL);
  if(res < 0)
    {
      wxLogMessage("client write error");
      _connected = false;
      res = 0;
    }
  return res;
}

int16_t BridgeClient::read()
{
  uint8_t data = 0;
  //wxLogMessage("read");
  int16_t res = read(&data, 1);
  if(res < 0)
    {
      wxLogMessage("client read error");
      return res;
    }
  return data;
}

int16_t BridgeClient::read(uint8_t *buf, size_t size)
{
  // wxLogMessage("client fd=%d read size %d\n", sockfd, size);
  if(!_connected)
    {
      //wxLogMessage("not connected");
      wxLogMessage("client fd=%d read error - not connected\n", sockfd);
      return -1;
    }
  //wxLogMessage("sock read %d\n", size);
  int16_t res = sock_read(sockfd, 0, 0);
  //wxLogMessage("sock read %d (%d)\n", res, size);
  if(size && res == 0 && available())
    {
      //wxLogMessage("recv...\n");
      res = recv(sockfd, buf, size, MSG_DONTWAIT | MSG_NOSIGNAL);
      //wxLogMessage("recv size %d (%d)\n", res, size);
    }
  if(res < 0)
    {
      wxLogMessage("client fd=%d sock/recv error\n", sockfd);
      _connected = false;
    }
  return res;
}

int16_t BridgeClient::available()
{
  int16_t count = 0;
  ioctl(sockfd, FIONREAD, &count);
  //wxLogMessage("available %d\n", count);
  return count;
}

void BridgeClient::stop()
{
  if(sockfd >= 0)
    {
      close(sockfd);
      //wxLogMessage("stopped client fd=%d\n", sockfd);
      sockfd = -1;
    }
  _connected = false;
}

uint8_t BridgeClient::connected()
{
  if (sockfd < 0 ) return 0;
  if(!_connected)
    {
      wxLogMessage("connected? client not connected - fd=%i",sockfd);
      return 0;
    }
  read(0,0);
  return _connected;
}

IPAddress BridgeClient::remoteIP(int16_t fd)
{
  struct sockaddr_storage addr;
  socklen_t len = sizeof addr;
  getpeername(fd, (struct sockaddr*)&addr, &len);
  struct sockaddr_in *s = (struct sockaddr_in *)&addr;
  return IPAddress((uint32_t)(s->sin_addr.s_addr));
}

uint16_t BridgeClient::remotePort(int16_t fd)
{
  struct sockaddr_storage addr;
  socklen_t len = sizeof addr;
  getpeername(fd, (struct sockaddr*)&addr, &len);
  struct sockaddr_in *s = (struct sockaddr_in *)&addr;
  return ntohs(s->sin_port);
}

IPAddress BridgeClient::remoteIP()
{
  return remoteIP(sockfd);
}

uint16_t BridgeClient::remotePort()
{
  return remotePort(sockfd);
}

bool BridgeClient::operator==(const BridgeClient& rhs)
{
  return sockfd == rhs.sockfd && remotePort(sockfd) == remotePort(rhs.sockfd) && remoteIP(sockfd) == remoteIP(rhs.sockfd);
}
