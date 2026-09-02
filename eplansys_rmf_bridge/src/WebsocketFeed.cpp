// Copyright 2026 Haniel Ulises
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "eplansys_rmf_bridge/WebsocketFeed.hpp"

#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

namespace eplansys_rmf_bridge
{

using Server = websocketpp::server<websocketpp::config::asio>;

class WebsocketFeed::Implementation
{
public:
  Implementation(int port, Callback callback)
  : callback_(std::move(callback))
  {
    server_.clear_access_channels(websocketpp::log::alevel::all);
    server_.clear_error_channels(websocketpp::log::elevel::all);
    server_.set_reuse_addr(true);
    server_.init_asio();

    server_.set_open_handler(
      [this](websocketpp::connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mutex_);
        connections_.insert(hdl);
      });

    server_.set_close_handler(
      [this](websocketpp::connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mutex_);
        connections_.erase(hdl);
      });

    server_.set_message_handler(
      [this](websocketpp::connection_hdl, Server::message_ptr msg) {
        on_message(msg->get_payload());
      });

    websocketpp::lib::error_code ec;
    server_.listen(port, ec);
    if (ec) {
      throw std::runtime_error(
              "cannot listen on port " + std::to_string(port) + ": " +
              ec.message());
    }
    server_.start_accept(ec);
    if (ec) {
      throw std::runtime_error("cannot accept connections: " + ec.message());
    }
  }

  ~Implementation()
  {
    stop();
  }

  void start()
  {
    if (thread_.joinable()) {
      return;
    }
    thread_ = std::thread([this]() {server_.run();});
  }

  void stop()
  {
    if (!thread_.joinable()) {
      return;
    }

    websocketpp::lib::error_code ec;
    server_.stop_listening(ec);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & hdl : connections_) {
        websocketpp::lib::error_code close_ec;
        server_.close(hdl, websocketpp::close::status::going_away, "", close_ec);
      }
      connections_.clear();
    }

    server_.stop();
    thread_.join();
  }

  std::size_t connections() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return connections_.size();
  }

  std::size_t received() const
  {
    return received_.load();
  }

private:
  void on_message(const std::string & payload)
  {
    received_.fetch_add(1);
    if (payload.empty()) {
      return;
    }

    // A frame that is not JSON is a frame to ignore. BroadcastClient opens by
    // sending a bare `Hello`, and letting that escape as an exception out of
    // a websocketpp handler takes the whole process down.
    const auto doc = nlohmann::json::parse(payload, nullptr, false);
    if (doc.is_discarded() || !doc.is_object()) {
      return;
    }

    try {
      callback_(doc);
    } catch (const std::exception &) {
      // Nothing this handler can usefully do, and throwing out of it aborts.
    }
  }

  Server server_;
  Callback callback_;
  std::thread thread_;

  mutable std::mutex mutex_;
  std::set<websocketpp::connection_hdl,
    std::owner_less<websocketpp::connection_hdl>> connections_;
  std::atomic<std::size_t> received_{0};
};

WebsocketFeed::WebsocketFeed(int port, Callback callback)
: impl_(std::make_unique<Implementation>(port, std::move(callback)))
{
}

WebsocketFeed::~WebsocketFeed() = default;

void WebsocketFeed::start()
{
  impl_->start();
}

void WebsocketFeed::stop()
{
  impl_->stop();
}

std::size_t WebsocketFeed::connections() const
{
  return impl_->connections();
}

std::size_t WebsocketFeed::received() const
{
  return impl_->received();
}

}  // namespace eplansys_rmf_bridge
