// Same-host CLOCK_MONOTONIC authority. No heartbeat, receipt-time renewal or re-arm.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_set>

namespace toponav {
struct Envelope {
  std::string session, kind, frame;
  std::uint64_t seq = 0;
  double captured = -1, issued = -1, expires = -1, v = 0, w = 0;
};
class ReceiverLease {
 public:
  // Per-session anti-replay history. Never evict evidence to admit more motion.
  // Covers the longest current task budget (2200 ticks); exhausting this budget
  // is a terminal fault, not a lease renewal or an automatic new session.
  static constexpr std::size_t kMaxCommandFrames = 4096;
  explicit ReceiverLease(std::string session) : session_(std::move(session)) {}
  void Trip(const std::string &why) {
    if (!latched) reason = why;
    latched = true; v = w = 0;
  }
  void Poll(double now) {
    if (!std::isfinite(now) || now < last_) { Trip("steady_clock_regression"); return; }
    last_ = now;
    if (!latched && seq && now >= deadline) Trip("command_lease_expired");
  }
  bool Accept(const Envelope &p, double now) {
    Poll(now);
    if (latched) return false;
    if (p.session != session_ || !std::isfinite(p.v) || !std::isfinite(p.w)
        || p.v < 0 || p.v > .15 || std::abs(p.w) > .30) {
      Trip("invalid_receiver_envelope"); return false;
    }
    if (p.kind == "stop" && p.v == 0 && p.w == 0) { Trip("remote_stop"); return true; }
    if (p.kind == "idle" && !seq && p.seq == 0 && p.v == 0 && p.w == 0) return true;
    if (p.kind != "command" || seq == std::numeric_limits<std::uint64_t>::max()
        || p.seq != seq+1 || p.frame.empty() || p.frame.size() > 256
        || frames_.count(p.frame) || !std::isfinite(p.captured) || p.captured < 0
        || p.captured <= captured_ || !std::isfinite(p.issued) || !std::isfinite(p.expires)
        || p.issued < 0 || p.issued > now || p.expires <= now
        || p.expires > p.issued+.8+1e-9) {
      Trip("invalid_receiver_envelope"); return false;
    }
    // Check BEFORE changing accepted authority/metadata or allocating a new
    // history entry. STOP/initial idle and envelope validation run first.
    if (frames_.size() >= kMaxCommandFrames) {
      Trip("receiver_capacity_exceeded"); return false;
    }
    deadline = std::min(p.expires, p.issued+.8);
    if (deadline <= now) { Trip("command_lease_expired"); return false; }
    seq = p.seq; captured_ = p.captured; frame_ = p.frame; issued_ = p.issued;
    frames_.insert(p.frame);
    v = p.v; w = p.w; reason = "accepted";
    return true;
  }
  bool latched = false;
  std::string reason = "disarmed";
  std::uint64_t seq = 0;
  double deadline = 0, v = 0, w = 0;
  // Last ACCEPTED command only. STOP, rejected packets and Poll never replace it.
  const std::string &Frame() const { return frame_; }
  double Captured() const { return captured_; }
  double Issued() const { return issued_; }
  std::size_t FrameCount() const { return frames_.size(); }
 private:
  std::string session_, frame_;
  double last_ = -1, captured_ = -1, issued_ = -1;
  std::unordered_set<std::string> frames_;
};
}  // namespace toponav
