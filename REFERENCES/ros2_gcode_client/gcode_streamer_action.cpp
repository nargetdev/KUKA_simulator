#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "gcode_streamer_action/action/stream_gcode.hpp"

namespace gcode_streamer_action
{

namespace
{

using namespace std::chrono_literals;

// Size of the binary command buffer sent to the robot.
constexpr std::size_t kBufferSize = 48;

// Simple representation of a motion command extracted from a G1 line.
struct MotionCommand
{
  std::optional<float> x;
  std::optional<float> y;
  std::optional<float> z;
  std::optional<float> a;
  std::optional<float> b;
  std::optional<float> c;
  std::optional<float> f;
  std::optional<float> e;
};

enum RobotMask : uint32_t
{
  kMaskX = 1u << 0,
  kMaskY = 1u << 1,
  kMaskZ = 1u << 2,
  kMaskA = 1u << 3,
  kMaskB = 1u << 4,
  kMaskC = 1u << 5,
  kMaskF = 1u << 6,
  kMaskE = 1u << 7,
};

struct RobotCommand
{
  int32_t seq{0};
  int32_t cmd_type{0};  // 0 == motion
  std::optional<float> x;
  std::optional<float> y;
  std::optional<float> z;
  std::optional<float> a;
  std::optional<float> b;
  std::optional<float> c;
  std::optional<float> f;
  std::optional<float> e;
  uint32_t bitmask{0};
};

template<typename T>
void write_little_endian(uint8_t * buffer, T value)
{
  std::memcpy(buffer, &value, sizeof(T));
}

void set_float_field(
  std::array<uint8_t, kBufferSize> & buffer,
  std::size_t offset,
  const std::optional<float> & value,
  uint32_t mask,
  uint32_t & bitmask)
{
  if (!value.has_value()) {
    return;
  }

  bitmask |= mask;
  static_assert(sizeof(float) == sizeof(uint32_t), "Unexpected float size");
  uint32_t raw = 0;
  std::memcpy(&raw, &(*value), sizeof(float));
  write_little_endian(buffer.data() + static_cast<long>(offset), raw);
}

std::array<uint8_t, kBufferSize> pack_command_buffer(const RobotCommand & command)
{
  std::array<uint8_t, kBufferSize> buffer{};

  write_little_endian(buffer.data(), static_cast<uint32_t>(command.seq));
  write_little_endian(buffer.data() + 4, static_cast<uint32_t>(command.cmd_type));

  uint32_t bitmask = command.bitmask;

  set_float_field(buffer, 8, command.x, kMaskX, bitmask);
  set_float_field(buffer, 12, command.y, kMaskY, bitmask);
  set_float_field(buffer, 16, command.z, kMaskZ, bitmask);
  set_float_field(buffer, 20, command.a, kMaskA, bitmask);
  set_float_field(buffer, 24, command.b, kMaskB, bitmask);
  set_float_field(buffer, 28, command.c, kMaskC, bitmask);
  set_float_field(buffer, 32, command.f, kMaskF, bitmask);
  set_float_field(buffer, 36, command.e, kMaskE, bitmask);

  write_little_endian(buffer.data() + 40, bitmask);

  return buffer;
}

// Very small TCP client with persistent connection and optional sequence reads.
class TcpClient
{
public:
  TcpClient(const std::string & host, uint16_t port)
  : host_(host), port_(port), socket_fd_(-1) {}

  ~TcpClient()
  {
    close_socket();
  }

  bool connect_socket(std::string & error)
  {
    if (socket_fd_ >= 0) {
      return true;
    }

    socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
      error = std::string("socket() failed: ") + std::strerror(errno);
      return false;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);

    if (::inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) <= 0) {
      error = "inet_pton() failed for host: " + host_;
      close_socket();
      return false;
    }

    if (::connect(
        socket_fd_,
        reinterpret_cast<sockaddr *>(&server_addr),
        sizeof(server_addr)) < 0)
    {
      error = std::string("connect() failed: ") + std::strerror(errno);
      close_socket();
      return false;
    }

    return true;
  }

  bool send_buffer(const std::array<uint8_t, kBufferSize> & buffer, std::string & error)
  {
    if (socket_fd_ < 0) {
      error = "socket not connected";
      return false;
    }

    const ssize_t sent = ::send(socket_fd_, buffer.data(), buffer.size(), 0);
    if (sent != static_cast<ssize_t>(buffer.size())) {
      error = std::string("send() failed: ") + std::strerror(errno);
      close_socket();
      return false;
    }

    return true;
  }

  // Try to read a feedback packet from the robot containing:
  //   - int32 seq
  //   - int32 queue_size
  // packed in little-endian format in the first 8 bytes of a 16-byte buffer.
  // Returns true if a packet was read, false if no data was available before
  // the timeout. Returns false and sets error on socket errors.
  bool receive_feedback(
    int32_t & seq,
    int32_t & queue_size,
    int timeout_ms,
    std::string & error)
  {
    if (socket_fd_ < 0) {
      error = "socket not connected";
      return false;
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(socket_fd_, &readfds);

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    const int ret = ::select(socket_fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      error = std::string("select() failed: ") + std::strerror(errno);
      close_socket();
      return false;
    }

    if (ret == 0 || !FD_ISSET(socket_fd_, &readfds)) {
      // Timeout with no data – this is not an error, just no update.
      return false;
    }

    // The robot sends 48 bytes; only the first 8 contain:
    //   [0..3] seq (uint32_t LE)
    //   [4..7] queue_size (uint32_t LE)
    // The remaining bytes are filler (-27) and are ignored here.
    std::array<uint8_t, 48> buf{};
    std::size_t total_received = 0;

    while (total_received < buf.size()) {
      const ssize_t received = ::recv(
        socket_fd_,
        buf.data() + static_cast<long>(total_received),
        buf.size() - total_received,
        0);
      if (received < 0) {
        error = std::string("recv() failed: ") + std::strerror(errno);
        close_socket();
        return false;
      }
      if (received == 0) {
        error = "recv() returned 0 bytes (peer closed connection)";
        close_socket();
        return false;
      }
      total_received += static_cast<std::size_t>(received);
    }

    uint32_t raw_seq = 0;
    uint32_t raw_qsize = 0;
    static_assert(sizeof(raw_seq) == 4, "Unexpected uint32_t size");
    std::memcpy(&raw_seq, buf.data(), sizeof(raw_seq));
    std::memcpy(&raw_qsize, buf.data() + 4, sizeof(raw_qsize));

    seq = static_cast<int32_t>(raw_seq);
    queue_size = static_cast<int32_t>(raw_qsize);
    return true;
  }

  void close_socket()
  {
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  bool is_connected() const
  {
    return socket_fd_ >= 0;
  }

private:
  std::string host_;
  uint16_t port_;
  int socket_fd_;
};

// Parse a single G1 line into a MotionCommand.
// Returns std::nullopt if the line is empty or comment-only.
// Returns std::optional with error string on parse failure.
struct ParseResult
{
  std::optional<MotionCommand> command;
  std::string error;
};

ParseResult parse_g1_line(const std::string & line)
{
  ParseResult result;

  // Trim leading/trailing whitespace.
  const auto first = line.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    // Empty line.
    return result;
  }
  if (line[first] == ';') {
    // Comment-only line.
    return result;
  }

  // Require that this is a G1 command; ignore other commands for now.
  // We look for "G1" as a standalone token (optionally with parameters after).
  std::size_t pos = first;
  if (line.compare(pos, 2, "G1") != 0 && line.compare(pos, 3, "G01") != 0) {
    result.error = "Only G1/G01 commands are supported";
    return result;
  }

  MotionCommand cmd;

  // Scan for parameters: X/Y/Z/A/B/C/F/E followed by a float.
  // Simple manual parsing is enough for now.
  pos = line.find_first_of(" \t", pos);
  while (pos != std::string::npos && pos < line.size()) {
    // Skip whitespace.
    while (pos < line.size() && (line[pos] == ' ' || line[pos] == '\t')) {
      ++pos;
    }
    if (pos >= line.size()) {
      break;
    }

    const char designator = line[pos];
    if (designator == ';') {
      // Rest of the line is a comment.
      break;
    }

    // Parameter value starts right after the designator.
    std::size_t value_start = pos + 1;
    std::size_t value_end = value_start;
    while (value_end < line.size() &&
      line[value_end] != ' ' &&
      line[value_end] != '\t' &&
      line[value_end] != ';')
    {
      ++value_end;
    }

    if (value_start == value_end) {
      result.error = "Missing value for parameter";
      return result;
    }

    const std::string value_str = line.substr(value_start, value_end - value_start);
    char * end_ptr = nullptr;
    const float value = std::strtof(value_str.c_str(), &end_ptr);
    if (end_ptr == value_str.c_str()) {
      result.error = "Failed to parse float value for parameter";
      return result;
    }

    switch (designator) {
      case 'X':
        cmd.x = value;
        break;
      case 'Y':
        cmd.y = value;
        break;
      case 'Z':
        cmd.z = value;
        break;
      case 'A':
        cmd.a = value;
        break;
      case 'B':
        cmd.b = value;
        break;
      case 'C':
        cmd.c = value;
        break;
      case 'F':
        cmd.f = value;
        break;
      case 'E':
        cmd.e = value;
        break;
      default:
        // Unknown designator – ignore for now.
        break;
    }

    pos = value_end;
  }

  result.command = cmd;
  return result;
}

}  // namespace

class GcodeStreamerActionServer : public rclcpp::Node
{
public:
  using StreamGcode = gcode_streamer_action::action::StreamGcode;
  using GoalHandleStreamGcode = rclcpp_action::ServerGoalHandle<StreamGcode>;

  GcodeStreamerActionServer()
  : Node("gcode_streamer_action_server")
  {
    declare_parameter<std::string>("robot_host", "172.31.1.147");
    declare_parameter<int>("robot_port", 54601);
    declare_parameter<int>("queue_size_target", 4);
    declare_parameter<double>("send_frequency_hz", 32.0);

    robot_host_ = get_parameter("robot_host").as_string();
    robot_port_ = static_cast<uint16_t>(get_parameter("robot_port").as_int());
    queue_size_target_ = get_parameter("queue_size_target").as_int();
    send_frequency_hz_ = get_parameter("send_frequency_hz").as_double();

    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<StreamGcode>(
      this,
      "gcode_streamer_action",
      std::bind(&GcodeStreamerActionServer::handle_goal, this, _1, _2),
      std::bind(&GcodeStreamerActionServer::handle_cancel, this, _1),
      std::bind(&GcodeStreamerActionServer::handle_accepted, this, _1));

    // Establish an initial TCP connection to the robot so that we are ready
    // before the first goal arrives.
    {
      std::string error;
      client_ = std::make_unique<TcpClient>(robot_host_, robot_port_);
      if (!client_->connect_socket(error)) {
        RCLCPP_WARN(
          get_logger(),
          "Initial connect to robot at %s:%u failed: %s (will retry on first goal)",
          robot_host_.c_str(),
          robot_port_,
          error.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Initial TCP connection to robot established at %s:%u",
          robot_host_.c_str(),
          robot_port_);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "gcode_streamer_action server ready, connecting to %s:%u (queue target=%d, freq=%.1f Hz)",
      robot_host_.c_str(),
      robot_port_,
      queue_size_target_,
      send_frequency_hz_);
  }

private:
  rclcpp_action::Server<StreamGcode>::SharedPtr action_server_;
  std::string robot_host_;
  uint16_t robot_port_{0};
  int queue_size_target_{4};
  double send_frequency_hz_{32.0};

  // Single persistent TCP client shared across goals. Access is serialized by
  // stream_mutex_ so only one goal uses it at a time.
  std::unique_ptr<TcpClient> client_;
  std::mutex stream_mutex_;

  // Ensure we have a connected client_ before executing a goal.
  bool ensure_client_connected(std::string & error)
  {
    if (!client_) {
      client_ = std::make_unique<TcpClient>(robot_host_, robot_port_);
    }
    if (client_->is_connected()) {
      return true;
    }
    return client_->connect_socket(error);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const StreamGcode::Goal> goal)
  {
    if (!goal) {
      RCLCPP_WARN(get_logger(), "Received null goal");
      return rclcpp_action::GoalResponse::REJECT;
    }

    const auto & lines = goal->gcode_lines.data;
    if (lines.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal with empty G-code line list");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
      get_logger(),
      "Accepted goal with %zu G-code lines",
      lines.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleStreamGcode> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleStreamGcode> goal_handle)
  {
    std::thread{std::bind(&GcodeStreamerActionServer::execute, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleStreamGcode> goal_handle)
  {
    // Serialize streaming so only one goal uses the TCP connection at a time.
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);

    RCLCPP_INFO(get_logger(), "Execute goal");
    const auto goal = goal_handle->get_goal();
    if (!goal) {
      RCLCPP_ERROR(get_logger(), "Goal handle has null goal");
      return;
    }

    const auto & lines = goal->gcode_lines.data;
    const std::size_t total_lines = lines.size();

    // Pre-parse all lines to validate before we start sending.
    std::vector<MotionCommand> commands;
    commands.reserve(total_lines);

    for (std::size_t i = 0; i < total_lines; ++i) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(get_logger(), "Goal canceled during validation");
        auto result = std::make_shared<StreamGcode::Result>();
        result->success = false;
        result->message = "Canceled during validation";
        result->lines_executed = 0;
        goal_handle->canceled(result);
        return;
      }

      const auto & line = lines[i];
      const ParseResult parse_result = parse_g1_line(line);

      if (!parse_result.error.empty()) {
        RCLCPP_ERROR(get_logger(), "Parse error at line %zu: %s", i, parse_result.error.c_str());
        auto result = std::make_shared<StreamGcode::Result>();
        result->success = false;
        result->message = "Parse error at line " + std::to_string(i) + ": " + parse_result.error;
        result->lines_executed = 0;
        goal_handle->abort(result);
        return;
      }

      if (!parse_result.command.has_value()) {
        // Comment or empty line – skip it and adjust total_lines in feedback later.
        continue;
      }

      commands.push_back(*parse_result.command);
    }

    const std::size_t effective_total_lines = commands.size();
    if (effective_total_lines == 0) {
      RCLCPP_WARN(get_logger(), "No executable G1 commands found in goal");
      auto result = std::make_shared<StreamGcode::Result>();
      result->success = false;
      result->message = "No executable G1 commands in input";
      result->lines_executed = 0;
      goal_handle->abort(result);
      return;
    }

    // Ensure persistent TCP connection to the robot.
    std::string error;
    if (!ensure_client_connected(error)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to connect to robot at %s:%u: %s",
        robot_host_.c_str(),
        robot_port_,
        error.c_str());
      auto result = std::make_shared<StreamGcode::Result>();
      result->success = false;
      result->message = "Failed to connect to robot: " + error;
      result->lines_executed = 0;
      goal_handle->abort(result);
      return;
    }

    TcpClient & client = *client_;

    RCLCPP_INFO(
      get_logger(),
      "Connected to robot at %s:%u, streaming %zu G1 commands",
      robot_host_.c_str(),
      robot_port_,
      effective_total_lines);

    rclcpp::Rate rate(send_frequency_hz_);

    int32_t last_sent_seq = 0;
    int32_t last_robot_seq = -1;
    int32_t last_robot_queue_size = -1;
    uint32_t lines_executed = 0;
    uint32_t seq_update_count = 0;

    for (std::size_t i = 0; i < effective_total_lines && rclcpp::ok(); ++i) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(get_logger(), "Goal canceled during execution");
        client.close_socket();
        auto result = std::make_shared<StreamGcode::Result>();
        result->success = false;
        result->message = "Canceled during execution";
        result->lines_executed = lines_executed;
        goal_handle->canceled(result);
        return;
      }

      // Wait until there is space in the robot queue.
      // We derive queue size as (last_sent_seq - last_robot_seq).
      // If we have not yet received any robot sequence, we optimistically send
      // the first few commands up to the queue_size_target_.
      while (rclcpp::ok()) {
        if (last_robot_seq >= 0) {
          const int32_t current_queue_size = last_sent_seq - last_robot_seq;
          if (current_queue_size < queue_size_target_) {
            break;
          }
        } else {
          // No feedback yet; allow up to queue_size_target_ commands to be sent.
          if (last_sent_seq < queue_size_target_) {
            break;
          }
        }

        int32_t seq_update = -1;
        int32_t qsize_update = -1;
        std::string recv_error;
        const bool got_seq = client.receive_feedback(seq_update, qsize_update, 50, recv_error);
        if (!recv_error.empty()) {
          RCLCPP_ERROR(
            get_logger(),
            "Error receiving sequence from robot: %s",
            recv_error.c_str());
          client.close_socket();
          auto result = std::make_shared<StreamGcode::Result>();
          result->success = false;
          result->message = "Error receiving sequence from robot: " + recv_error;
          result->lines_executed = lines_executed;
          goal_handle->abort(result);
          return;
        }
        if (got_seq) {
          last_robot_seq = seq_update;
          last_robot_queue_size = qsize_update;
          ++seq_update_count;
          RCLCPP_INFO(
            get_logger(),
            "Robot feedback: seq=%d queue_size=%d (update #%u, during wait)",
            seq_update,
            qsize_update,
            seq_update_count);
        }
      }

      if (!rclcpp::ok()) {
        client.close_socket();
        auto result = std::make_shared<StreamGcode::Result>();
        result->success = false;
        result->message = "ROS shutdown during execution";
        result->lines_executed = lines_executed;
        goal_handle->abort(result);
        return;
      }

      // Build RobotCommand for this motion.
      const MotionCommand & motion = commands[i];
      RobotCommand command;
      command.seq = ++last_sent_seq;
      command.cmd_type = 0;  // motion
      command.x = motion.x;
      command.y = motion.y;
      command.z = motion.z;
      command.a = motion.a;
      command.b = motion.b;
      command.c = motion.c;
      command.f = motion.f;
      command.e = motion.e;

      auto buffer = pack_command_buffer(command);

      if (!client.send_buffer(buffer, error)) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to send command (seq=%d): %s",
          command.seq,
          error.c_str());
        client.close_socket();
        auto result = std::make_shared<StreamGcode::Result>();
        result->success = false;
        result->message = "Failed to send command: " + error;
        result->lines_executed = lines_executed;
        goal_handle->abort(result);
        return;
      }

      ++lines_executed;

      // Update robot sequence if an acknowledgement is available.
      {
        int32_t seq_update = -1;
        int32_t qsize_update = -1;
        std::string recv_error;
        const bool got_seq = client.receive_feedback(seq_update, qsize_update, 10, recv_error);
        if (!recv_error.empty()) {
          RCLCPP_ERROR(
            get_logger(),
            "Error receiving sequence from robot after send: %s",
            recv_error.c_str());
          client.close_socket();
          auto result = std::make_shared<StreamGcode::Result>();
          result->success = false;
          result->message = "Error receiving sequence from robot: " + recv_error;
          result->lines_executed = lines_executed;
          goal_handle->abort(result);
          return;
        }
        if (got_seq) {
          last_robot_seq = seq_update;
          last_robot_queue_size = qsize_update;
          ++seq_update_count;
          RCLCPP_INFO(
            get_logger(),
            "Robot feedback: seq=%d queue_size=%d (update #%u, after send)",
            seq_update,
            qsize_update,
            seq_update_count);
        }
      }

      // Publish feedback.
      auto feedback = std::make_shared<StreamGcode::Feedback>();
      // Prefer the queue size reported by the robot if available; otherwise
      // fall back to our derived estimate.
      int32_t queue_size_current =
        (last_robot_queue_size >= 0) ? last_robot_queue_size :
        ((last_robot_seq >= 0) ? (last_sent_seq - last_robot_seq) :
         static_cast<int32_t>(last_sent_seq));
      feedback->queue_size_current =
        queue_size_current >= 0 ? static_cast<uint32_t>(queue_size_current) : 0;
      feedback->queue_size_target = static_cast<uint32_t>(queue_size_target_);
      feedback->lines_executed = lines_executed;
      feedback->lines_total = static_cast<uint32_t>(effective_total_lines);
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    client.close_socket();

    RCLCPP_INFO(
      get_logger(),
      "Completed streaming %u/%zu G1 commands",
      lines_executed,
      effective_total_lines);

    auto result = std::make_shared<StreamGcode::Result>();
    result->success = true;
    result->message = "Completed streaming G1 commands";
    result->lines_executed = lines_executed;
    goal_handle->succeed(result);
  }
};

}  // namespace gcode_streamer_action

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gcode_streamer_action::GcodeStreamerActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

