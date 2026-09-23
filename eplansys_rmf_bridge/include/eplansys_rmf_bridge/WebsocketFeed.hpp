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

#ifndef EPLANSYS_RMF_BRIDGE__WEBSOCKETFEED_HPP_
#define EPLANSYS_RMF_BRIDGE__WEBSOCKETFEED_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

namespace eplansys_rmf_bridge
{

/// The websocket a fleet adapter is pointed at by its `server_uri` parameter.
///
/// On Humble this is the only way a task state or a task log leaves the
/// adapter; the ROS 2 mirror of those topics arrived later than this release.
///
/// `rmf_websocket::BroadcastServer` is the obvious thing to use here and does
/// not work. Measured against 2.1.8: an adapter reports the connection open,
/// publishes without error, and the callback is never invoked, while a plain
/// websocket server on the same port receives every frame from the same
/// adapter. It also aborts the process on the first non-JSON frame, since its
/// handler calls nlohmann::json::parse with exceptions enabled and
/// BroadcastClient opens by sending a bare `Hello` probe. So the server is our
/// own, and it treats an unparseable frame as a frame to ignore.
class WebsocketFeed
{
public:
  using Callback = std::function<void (const nlohmann::json &)>;

  /// Binds the port. Throws std::runtime_error if it cannot.
  WebsocketFeed(int port, Callback callback);
  ~WebsocketFeed();

  WebsocketFeed(const WebsocketFeed &) = delete;
  WebsocketFeed & operator=(const WebsocketFeed &) = delete;

  /// Starts the io thread.
  void start();

  /// Stops it and closes every connection.
  void stop();

  /// How many adapters are connected. A fleet adapter and the task dispatcher
  /// each open one.
  std::size_t connections() const;

  /// Frames accepted so far, whether or not they parsed.
  std::size_t received() const;

  /// Whether frames reach the callback.
  ///
  /// A fleet that stops answering is not the same thing as a fleet that has
  /// stopped: the robots keep their positions, the adapter keeps its state,
  /// and what is lost is the road back. Setting this false is that loss, and
  /// nothing else --- the socket stays up, the frames are still counted, and
  /// they are dropped instead of delivered. It exists so that an experiment
  /// can interrupt the link without killing the thing at the other end of it,
  /// which is the difference between measuring a recovery and measuring a
  /// restart.
  void deliver(bool yes);
  bool delivering() const;

  class Implementation;

private:
  std::unique_ptr<Implementation> impl_;
};

}  // namespace eplansys_rmf_bridge

#endif  // EPLANSYS_RMF_BRIDGE__WEBSOCKETFEED_HPP_
