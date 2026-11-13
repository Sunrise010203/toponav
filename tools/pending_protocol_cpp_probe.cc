// Opt-in single-document stdin/stdout wrapper; no transport, owner or runtime.
#include "pending_protocol_cpp_draft.hh"
#include <array>
#include <iostream>

int main(int argc,char **) {
  namespace draft=toponav_pending_offline;
  if (argc!=1) { std::cerr<<"no arguments; one offline document on stdin\n";return 64; }
  std::array<char,draft::MaxBytes+1> buffer{};
  std::cin.read(buffer.data(),static_cast<std::streamsize>(buffer.size()));
  draft::Json decoded;std::string error;
  try {
    draft::Need(!std::cin.bad(),"stdin_read_failed");
    decoded=draft::Decode(std::string_view(buffer.data(),static_cast<std::size_t>(std::cin.gcount())));
  } catch (const draft::Refusal &refusal) { error=refusal.what(); }
  draft::Json out=draft::Json::Obj();
  out.object={{"schema_consistent",draft::Json::Bool(error.empty())},{"decoded",decoded},
    {"error",error.empty()?draft::Json{}:draft::Json::Str(error)},
    {"classification",draft::Json::Str("offline_schema_not_admission")},{"actual_exposure_wall_s",{}}};
  for (const auto key:{"provenance_authenticated","exposure_age_bounded","shadow_admission_authorized",
      "native_live_observation_authorized","motion_authority","independent_enforcement_observed",
      "physical_stop_proved","production_guard_fixed","runtime_pending_installed","runtime_result_consumed"})
    out.object[key]=draft::Json::Bool(false);
  std::cout<<draft::Dump(out)<<'\n';return error.empty()?0:2;
}
