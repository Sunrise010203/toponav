// Opt-in standalone arithmetic helper. Original header only, NO Gazebo linkage.
// Closed grammar: poll ns | send ns seq frame captured_sim_ns issued_ns expires_ns.
// Caller supplies fictional times. Every envelope has v=w=0 by construction.
#include "simulation/native/receiver_lease.hh"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
std::uint64_t number(const std::string &s) {
  if (s.empty() || s.size() > 13 || s.find_first_not_of("0123456789") != std::string::npos)
    throw std::invalid_argument("integer");
  const auto n = std::stoull(s);
  if (n > 1000000000000ULL) throw std::invalid_argument("bound");
  return n;
}
double seconds(const std::string &s) { return static_cast<double>(number(s))/1e9; }
bool frame(const std::string &s) {
  return !s.empty() && s.size() <= 64 &&
    s.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") == std::string::npos;
}
}
int main() {
  toponav::ReceiverLease guard("synthetic-zero");
  std::string line;
  std::size_t rows = 0;
  try {
    while (std::getline(std::cin, line)) {
      if (++rows > 64 || line.size() > 1024) throw std::invalid_argument("row bound");
      std::istringstream in(line);
      std::vector<std::string> t;
      std::string token;
      while (in >> token) t.push_back(token);
      if (t.empty()) throw std::invalid_argument("empty row");
      bool accepted = false, send = t[0] == "send";
      if (t[0] == "poll" && t.size() == 2) guard.Poll(seconds(t[1]));
      else if (send && t.size() == 7 && frame(t[3])) {
        toponav::Envelope p;
        p.session = "synthetic-zero"; p.kind = "command";
        p.seq = number(t[2]);
        if (!p.seq || p.seq > 64) throw std::invalid_argument("sequence bound");
        p.frame = t[3]; p.captured = seconds(t[4]);
        p.issued = seconds(t[5]); p.expires = seconds(t[6]);
        p.v = 0.; p.w = 0.;
        accepted = guard.Accept(p, seconds(t[1]));
      } else throw std::invalid_argument("closed grammar");
      std::cout << std::setprecision(17) << "{\"accepted\":";
      if (send) std::cout << (accepted ? "true" : "false"); else std::cout << "null";
      std::cout << ",\"latched\":" << (guard.latched ? "true" : "false")
        << ",\"reason\":\"" << guard.reason << "\",\"seq\":" << guard.seq << ",\"deadline_steady_s\":";
      if (guard.seq) std::cout << guard.deadline; else std::cout << "null";
      std::cout << ",\"wheels\":[" << guard.v << ',' << guard.w << "]"
        << ",\"frame\":\"" << guard.Frame() << "\",\"issued_steady_s\":" << guard.Issued()
        << ",\"captured_sim_s\":" << guard.Captured() << "}\n";
    }
    if (!rows) throw std::invalid_argument("no rows");
  } catch (const std::exception &) {
    std::cerr << "invalid synthetic zero-only operation\n";
    return 2;
  }
}
