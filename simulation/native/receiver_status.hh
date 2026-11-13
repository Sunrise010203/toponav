// Status v2 echoes accepted envelope metadata, NOT image pixels or identity proof.
#pragma once
#include "receiver_lease.hh"
#include <iomanip>
#include <locale>
#include <sstream>

namespace toponav {
inline std::string ReceiverFrameHex(const std::string &frame) {
  // Preserve the original bytes, including quotes, controls, NUL and UTF-8.
  // Hex needs no JSON escaping and is bounded by the accepted 256-byte frame.
  constexpr char digits[] = "0123456789abcdef";
  std::string hex;
  hex.reserve(frame.size()*2);
  for (unsigned char c : frame) {
    hex += digits[c >> 4]; hex += digits[c & 15];
  }
  return hex;
}
inline std::string ReceiverStatus(const ReceiverLease &guard, const std::string &session,
    double now, double sim) {
  // session is ASCII-validated by Configure; reason is a native constant.
  std::ostringstream text;
  text.imbue(std::locale::classic());
  text << std::setprecision(17) << "{\"version\":2,\"session_id\":\"" << session
       << "\",\"wall_s\":" << (std::isfinite(now)?now:0)
       << ",\"sim_s\":" << sim
       << ",\"latched\":" << (guard.latched?"true":"false")
       << ",\"reason\":\"" << guard.reason << "\",\"command_seq\":" << guard.seq
       << ",\"deadline_steady_s\":" << guard.deadline
       << ",\"wheels\":[" << guard.v << ',' << guard.w << ']';
  if (guard.seq) {
    text << ",\"frame_id_hex\":\"" << ReceiverFrameHex(guard.Frame())
         << "\",\"captured_sim_s\":" << guard.Captured()
         << ",\"issued_steady_s\":" << guard.Issued();
  } else {
    text << ",\"frame_id_hex\":null,\"captured_sim_s\":null,\"issued_steady_s\":null";
  }
  text << '}';
  return text.str();
}
}  // namespace toponav
