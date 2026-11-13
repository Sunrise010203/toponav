// One bounded offline trace document; no CLI operation/transport/owner.
#include "pending_trace_cpp_review.hh"
#include <array>
#include <iostream>

int main(int argc,char **) {
  namespace d=toponav_pending_trace_offline;
  if (argc!=1) { std::cerr<<"no arguments; one offline trace on stdin\n";return 64; }
  std::array<char,d::TraceMaxBytes+1> buffer{};
  std::cin.read(buffer.data(),static_cast<std::streamsize>(buffer.size()));
  d::Json result;std::string error;
  try {
    d::Need(!std::cin.bad(),"stdin_read_failed");
    const auto raw=std::string_view(buffer.data(),static_cast<std::size_t>(std::cin.gcount()));
    const auto review=d::ReviewTrace(raw);const auto repeated=d::ReviewTrace(raw);
    result=d::Json::Obj();result.object={{"review",review},{"repeated_review",repeated}};
  } catch (const d::Refusal &refusal) { error=refusal.what(); }
  auto out=d::Review("offline_transcript_not_admission",d::Json{},false);
  out.object["runtime_one_time_consumption_enforced"]=d::Json::Bool(false);out.object["cross_run_replay_prevented"]=d::Json::Bool(false);
  out.object["schema_valid"]=d::Json::Bool(error.empty());out.object["result"]=result;
  out.object["error"]=error.empty()?d::Json{}:d::Json::Str(error);
  const bool consistent=error.empty() && result.at("review").at("consistent").boolean;
  out.object["consistent"]=d::Json::Bool(consistent);
  std::cout<<d::TraceDump(out)<<'\n';return error.empty()?(consistent?0:3):2;
}
