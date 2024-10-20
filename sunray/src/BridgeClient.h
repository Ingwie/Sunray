/*
  Client.h - Base class that provides Client
*/

#ifndef pi_client_h
#define pi_client_h

#include "Client.h"

class BridgeServer;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"

class BridgeClient : public Client
{
protected:
  BridgeServer *bridgeServer;
  int16_t sockfd;
  bool _connected;
  struct hostent *server;
public:
  BridgeClient *next;
  BridgeClient():bridgeServer(NULL), sockfd(-1),_connected(false),next(NULL),server(NULL) {}
  BridgeClient(BridgeServer *aServer):bridgeServer(aServer), sockfd(-1),_connected(false),next(NULL),server(NULL) {}
  BridgeClient(BridgeServer *aServer, int16_t fd):bridgeServer(aServer), sockfd(fd),_connected(true),next(NULL),server(NULL)
  {
    //Serial.print("new client fd=");
    //Serial.print(sockfd);
    //Serial.print("  connected=");
    //Serial.println(_connected);
  }
  ~BridgeClient();
  virtual int16_t connect(IPAddress ip, uint16_t port);
  virtual int16_t connect(const char *host, uint16_t port);
  virtual size_t write(uint8_t data);
  virtual size_t write(const uint8_t *buf, size_t size);
  virtual int16_t available();
  virtual int16_t read();
  virtual int16_t read(uint8_t *buf, size_t size);
  virtual int16_t peek()
  {
    return 0;
  }
  virtual void flush() {}
  virtual void stop();
  virtual uint8_t connected();
  virtual operator bool()
  {
    return connected();
  }
  virtual bool operator==(const bool value)
  {
    return bool() == value;
  }
  virtual bool operator!=(const bool value)
  {
    return bool() != value;
  }
  virtual bool operator==(const BridgeClient&);
  virtual bool operator!=(const BridgeClient& rhs)
  {
    return !this->operator==(rhs);
  };

  int16_t fd()
  {
    return sockfd;
  }
  IPAddress remoteIP();
  uint16_t remotePort();
  int16_t setSocketOption(int16_t option, char* value, size_t len);
  int16_t setOption(int16_t option, int16_t *value);
  int16_t getOption(int16_t option, int16_t *value);
  int16_t setTimeout(uint32_t seconds);
  int16_t setNoDelay(bool nodelay);
  bool getNoDelay();

  static IPAddress remoteIP(int16_t fd);
  static uint16_t remotePort(int16_t fd);

  friend class BridgeServer;
  using Print::write;
};
#pragma GCC diagnostic pop

#define WiFiEspClient BridgeClient
#define WiFiClient BridgeClient

#endif

