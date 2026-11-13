// Diagnostic ONLY: real ReceiverLease, private socket, zero commands, no Gazebo.
// The 5ms test polling loop is NOT the plugin's PreUpdate scheduling guarantee.
#include "simulation/native/receiver_lease.hh"
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

static double Now() {
  timespec t{};
  if (clock_gettime(CLOCK_MONOTONIC, &t)) std::abort();
  return double(t.tv_sec)+t.tv_nsec/1e9;
}
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const int fd = std::stoi(argv[1]);
  toponav::ReceiverLease gate("timing-audit");
  auto log = [&](const char *event, double received = -1.) {
    std::cout << std::setprecision(17) << "{\"event\":\"" << event
      << "\",\"wall_s\":" << Now() << ",\"received_steady_s\":" << received
      << ",\"reason\":\"" << gate.reason << "\",\"latched\":" << (gate.latched?"true":"false")
      << ",\"command_seq\":" << gate.seq << ",\"deadline_steady_s\":" << gate.deadline
      << ",\"wheels\":[" << gate.v << ',' << gate.w << "]}" << std::endl;
  };
  log("ready");
  if (send(fd, "ready", 5, MSG_NOSIGNAL) != 5) return 3;
  for (;;) {
    const bool before = gate.latched;
    gate.Poll(Now());
    if (!before && gate.latched) log("deadline_latched");
    fd_set read; FD_ZERO(&read); FD_SET(fd, &read);
    timeval wait{0, 5000};
    const int ready = select(fd+1, &read, nullptr, nullptr, &wait);
    if (ready < 0) return 4;
    if (!ready) continue;
    char buffer[4096];
    const auto size = recv(fd, buffer, sizeof(buffer), 0);
    const double received = Now();
    if (size <= 0) break;
    const std::string raw(buffer, size);
    if (raw == "quit") break;
    toponav::Envelope p;
    std::istringstream wire(raw);
    // Private diagnostic encoding, NOT the native Twist/DecodeEnvelope path.
    wire >> p.session >> p.kind >> p.frame >> p.seq >> p.captured
         >> p.issued >> p.expires >> p.v >> p.w;
    if (wire.fail() || !(wire >> std::ws).eof() || p.v != 0 || p.w != 0) {
      gate.Trip("invalid_receiver_envelope");
    } else {
      gate.Accept(p, received);
    }
    log("received", received);
  }
  gate.Trip("remote_stop");
  log("exit");
  close(fd);
  return 0;
}
