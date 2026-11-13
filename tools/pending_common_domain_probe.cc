// Standalone OFFLINE adapter only. No runtime native transport or device IO.
#include "pending_common_domain.hh"
#include <iostream>
int main(int argc,char **) {
  namespace c=toponav_pending_common_v1;namespace d=toponav_pending_offline;
  if (argc!=1) { std::cerr<<"no arguments; offline TPT1 only\n";return 64; }
  std::array<char,c::Limit+1> buffer{};
  std::cin.read(buffer.data(),static_cast<std::streamsize>(buffer.size()));
  if (std::cin.bad()) { std::cerr<<"offline input failure\n";return 74; }
  const auto out=c::ReviewEnvelope(std::string_view(buffer.data(),static_cast<std::size_t>(std::cin.gcount())));
  std::cout<<d::Dump(out)<<'\n';return out.at("consistent").boolean?0:2;
}
