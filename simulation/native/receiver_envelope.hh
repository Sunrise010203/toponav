// Bounded application parsing AFTER transport/protobuf decoding. This cannot
// bound deserialization, middleware queues, OS scheduling or other plugins.
#pragma once
#include "receiver_lease.hh"
#include <array>
#include <charconv>
#include <optional>
#include <string_view>

namespace toponav {
template<typename T> bool ParseReceiverNumber(std::string_view s, T &value) {
  if (s.empty()) return false;
  const auto result = std::from_chars(s.data(), s.data()+s.size(), value);
  return result.ec == std::errc() && result.ptr == s.data()+s.size();
}

// Template only to permit instrumented protobuf-shaped views in portable tests;
// production instantiates this same decoder with gz::msgs::Twist.
template<typename Twist> std::optional<Envelope> DecodeEnvelope(const Twist &msg) {
  constexpr std::array<std::string_view, 8> keys = {
    "version", "session_id", "kind", "command_seq", "frame_id",
    "captured_sim_s", "issued_steady_s", "expires_steady_s"};
  constexpr std::array<std::size_t, 8> limits = {1, 256, 7, 20, 256, 64, 64, 64};
  // Check cardinality in O(1), BEFORE iteration, comparisons or string copies.
  if (msg.header().data_size() != 8 || msg.linear().y() != 0 || msg.linear().z() != 0
      || msg.angular().x() != 0 || msg.angular().y() != 0) return std::nullopt;
  std::array<std::string_view, 8> fields{};
  unsigned seen = 0;
  for (int n = 0; n < 8; ++n) {
    const auto &entry = msg.header().data(n);
    if (entry.key().size() > 32 || entry.value_size() != 1) return std::nullopt;
    std::size_t index = 0;
    while (index < keys.size() && entry.key() != keys[index]) ++index;
    if (index == keys.size() || (seen & (1u << index))) return std::nullopt;
    const auto &value = entry.value(0);
    if (value.size() > limits[index]) return std::nullopt;
    fields[index] = value;
    seen |= 1u << index;
  }
  Envelope p;
  if (fields[0] != "1" || !ParseReceiverNumber(fields[3], p.seq)
      || !ParseReceiverNumber(fields[5], p.captured)
      || !ParseReceiverNumber(fields[6], p.issued)
      || !ParseReceiverNumber(fields[7], p.expires)) return std::nullopt;
  // At most 519 string bytes copied, only AFTER every field is bounded.
  p.session = fields[1]; p.kind = fields[2]; p.frame = fields[4];
  p.v = msg.linear().x(); p.w = msg.angular().z();
  return p;
}
}  // namespace toponav
