#include "xela_tactile/websocket_client.hpp"

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

ConnectionMetadata::ConnectionMetadata(int id, websocketpp::connection_hdl hdl, std::string uri)
  : m_id_(id)
  , m_hdl_(hdl)
  , m_status_("Connecting")
  , m_uri_(uri)
  , m_server_("N/A")
{
}

void ConnectionMetadata::on_open(client* c, websocketpp::connection_hdl hdl)
{
  m_status_ = "Open";

  client::connection_ptr con = c->get_con_from_hdl(hdl);
  m_server_ = con->get_response_header("Server");
}

void ConnectionMetadata::on_fail(client* c, websocketpp::connection_hdl hdl)
{
  m_status_ = "Failed";

  client::connection_ptr con = c->get_con_from_hdl(hdl);
  m_server_ = con->get_response_header("Server");
  m_error_reason_ = con->get_ec().message();
}

void ConnectionMetadata::on_close(client* c, websocketpp::connection_hdl hdl)
{
  m_status_ = "Closed";

  client::connection_ptr con = c->get_con_from_hdl(hdl);
  std::stringstream s;
  s << "close code: " << con->get_remote_close_code() << " ("
    << websocketpp::close::status::get_string(con->get_remote_close_code())
    << "), close reason: " << con->get_remote_close_reason();
  m_error_reason_ = s.str();
}
void ConnectionMetadata::on_message(websocketpp::connection_hdl, client::message_ptr msg)
{
  if (msg->get_opcode() == websocketpp::frame::opcode::text)
  {
    m_message_ = msg->get_payload();
  }
  else
  {
    m_message_ = websocketpp::utility::to_hex(msg->get_payload());
  }
}

websocketpp::connection_hdl ConnectionMetadata::get_hdl() const
{
  return m_hdl_;
}

int ConnectionMetadata::get_id() const
{
  return m_id_;
}

std::string ConnectionMetadata::get_status() const
{
  return m_status_;
}

std::string ConnectionMetadata::get_message()
{
  return m_message_;
}

WebSocketClient::WebSocketClient()
  : m_next_id_(0)
{
  m_endpoint_.clear_access_channels(websocketpp::log::alevel::all);
  m_endpoint_.clear_error_channels(websocketpp::log::elevel::all);

  m_endpoint_.init_asio();
  m_endpoint_.start_perpetual();
  m_thread_ = websocketpp::lib::make_shared<websocketpp::lib::thread>(&client::run, &m_endpoint_);
}

WebSocketClient::~WebSocketClient()
{
  m_endpoint_.stop_perpetual();

  for (con_list::const_iterator it = m_connection_list_.begin(); it != m_connection_list_.end(); ++it)
  {
    if (it->second->get_status() != "Open")
    {
      // Only close open connections
      continue;
    }

    std::cout << "> Closing connection " << it->second->get_id() << std::endl;

    websocketpp::lib::error_code ec;
    m_endpoint_.close(it->second->get_hdl(), websocketpp::close::status::going_away, "", ec);
    if (ec)
    {
      std::cout << "> Error closing connection " << it->second->get_id() << ": "
                << ec.message() << std::endl;
    }
  }

  m_thread_->join();
}

void WebSocketClient::close(int id, websocketpp::close::status::value code, std::string reason)
{
  websocketpp::lib::error_code ec;

  con_list::iterator metadata_it = m_connection_list_.find(id);
  if (metadata_it == m_connection_list_.end())
  {
    std::cout << "> No connection found with id " << id << std::endl;
    return;
  }

  m_endpoint_.close(metadata_it->second->get_hdl(), code, reason, ec);
  if (ec)
  {
    std::cout << "> Error initiating close: " << ec.message() << std::endl;
  }
}
int WebSocketClient::connect(std::string const & uri)
{
  websocketpp::lib::error_code ec;

  client::connection_ptr con = m_endpoint_.get_connection(uri, ec);

  if (ec)
  {
    std::cout << "> Connect initialization error: " << ec.message() << std::endl;
    return -1;
  }

  int new_id = m_next_id_++;
  ConnectionMetadata::ptr metadata_ptr
    = websocketpp::lib::make_shared<ConnectionMetadata>(new_id, con->get_handle(), uri);
  m_connection_list_[new_id] = metadata_ptr;

  con->set_open_handler(websocketpp::lib::bind(
     &ConnectionMetadata::on_open,
     metadata_ptr,
     &m_endpoint_,
     websocketpp::lib::placeholders::_1
  ));
  con->set_fail_handler(websocketpp::lib::bind(
     &ConnectionMetadata::on_fail,
     metadata_ptr,
     &m_endpoint_,
     websocketpp::lib::placeholders::_1
  ));
  con->set_close_handler(websocketpp::lib::bind(
     &ConnectionMetadata::on_close,
     metadata_ptr,
     &m_endpoint_,
     websocketpp::lib::placeholders::_1
  ));
  con->set_message_handler(websocketpp::lib::bind(
     &ConnectionMetadata::on_message,
     metadata_ptr,
     websocketpp::lib::placeholders::_1,
     websocketpp::lib::placeholders::_2
  ));

  m_endpoint_.connect(con);

  return new_id;
}

ConnectionMetadata::ptr WebSocketClient::get_metadata(int id) const
{
  con_list::const_iterator metadata_it = m_connection_list_.find(id);
  if (metadata_it == m_connection_list_.end())
  {
    return ConnectionMetadata::ptr();
  } else
  {
    return metadata_it->second;
  }
}

