#pragma once

#include <map>
#include <string>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

typedef websocketpp::client<websocketpp::config::asio_client> client;

class ConnectionMetadata
{
public:
  typedef websocketpp::lib::shared_ptr<ConnectionMetadata> ptr;
  ConnectionMetadata(int id, websocketpp::connection_hdl hdl, std::string uri);

  void on_open(client* c, websocketpp::connection_hdl hdl);
  void on_fail(client* c, websocketpp::connection_hdl hdl);
  void on_close(client* c, websocketpp::connection_hdl hdl);
  void on_message(websocketpp::connection_hdl, client::message_ptr msg);
  websocketpp::connection_hdl get_hdl() const;
  int get_id() const;
  std::string get_status() const;
  std::string get_message();
private:
  int m_id_;
  websocketpp::connection_hdl m_hdl_;
  std::string m_status_;
  std::string m_uri_;
  std::string m_server_;
  std::string m_error_reason_;
  std::string m_message_;
};

class WebSocketClient
{
public:
  WebSocketClient();
  ~WebSocketClient();

  int connect(std::string const& uri);
  void close(int id, websocketpp::close::status::value code, std::string reason);
  ConnectionMetadata::ptr get_metadata(int id) const;
private:
  typedef std::map<int, ConnectionMetadata::ptr> con_list;
  client m_endpoint_;
  websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread_;

  con_list m_connection_list_;
  int m_next_id_;
};
