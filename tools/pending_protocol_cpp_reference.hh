// PURE OFFLINE caller-snapshot review. No registry, owner, admission or execution.
#pragma once
#include "pending_protocol_cpp_draft.hh"
#include <optional>

namespace toponav_pending_offline {
using RawContext=std::optional<std::string_view>;
inline bool Same(const Json &a,const Json &b) { return Dump(a)==Dump(b); }
inline Json Context(RawContext raw) {
  Need(raw.has_value(),"message_byte_bound");return Decode(*raw);
}
inline Json Review(const char *classification,const Json &kind,bool matches,const Json &backend={}) {
  Json out=Json::Obj();out.object={{"classification",Json::Str(classification)},
    {"message_kind",kind},{"synthetic_reference_matches",Json::Bool(matches)},
    {"backend_claim",backend},{"actual_exposure_wall_s",{}}};
  for (const auto key:{"provenance_authenticated","exposure_age_bounded","shadow_admission_authorized",
      "native_live_observation_authorized","motion_authority","independent_enforcement_observed",
      "physical_stop_proved","production_guard_fixed","runtime_pending_installed","runtime_result_consumed"})
    out.object[key]=Json::Bool(false);
  return out;
}
inline void Reference(const Json &message,const Json &reserve,const Json &seal,
                      const Json &configured,bool frozen) {
  Need(frozen,"frozen_view_config");Views(configured);
  const auto &kind=message.at("kind");
  Need((Eq(kind,"reserve") || Eq(kind,"seal") || Eq(kind,"result_ref")) && Eq(reserve.at("kind"),"reserve"),"reference_kind");
  Need(Same(message.at("session_id"),reserve.at("session_id")) && Same(message.at("record"),reserve.at("record")),"original_record_reference");
  Need(Same(reserve.at("record").at("required_views"),configured),"frozen_view_config_mismatch");
  if (Eq(kind,"reserve")) Need(Same(message.at("request_id"),reserve.at("request_id")),"reserve_request_reference");
  else Need(message.at("request_id").integer>reserve.at("request_id").integer,"request_order");
  if (Eq(kind,"result_ref")) {
    Need(seal.type==Json::Object && Eq(seal.at("kind"),"seal"),"seal_reference_required");
    Reference(seal,reserve,Json{},configured,frozen);
    Need(Same(message.at("view_digests"),seal.at("view_digests")) &&
      Same(message.at("seal_request_id"),seal.at("request_id")) &&
      message.at("request_id").integer>seal.at("request_id").integer,"sealed_result_reference");
  }
}
inline Json ReviewReference(std::string_view raw,RawContext reserveRaw,RawContext sealRaw,
                            const Json &configured,bool frozen) {
  const auto message=Decode(raw);
  if (Eq(message.at("kind"),"stop")) return Review("frame_free_stop_schema_only",message.at("kind"),true);
  const auto reserve=Context(reserveRaw);const auto seal=sealRaw?Decode(*sealRaw):Json{};
  Reference(message,reserve,seal,configured,frozen);
  return Review("caller_snapshot_reference_only",message.at("kind"),true);
}
inline Json ReviewReply(std::string_view raw,std::string_view replyRaw,RawContext reserveRaw,
    RawContext sealRaw,const Json &configured,bool frozen,const Json &backend,
    const Json &reviewNs,const Json &oldHold) {
  const auto request=Decode(raw);const auto reply=Decode(replyRaw);const auto &kind=request.at("kind");
  Need((Eq(kind,"reserve") || Eq(kind,"seal") || Eq(kind,"result_ref") || Eq(kind,"stop")) &&
       (Eq(reply.at("kind"),"ack") || Eq(reply.at("kind"),"status")),"reply_kind");
  const auto &snapshot=reply.at("snapshot");
  Need((Eq(backend,"python") || Eq(backend,"native")) && Same(snapshot.at("backend"),backend),"reply_backend");
  Need(UInt(reviewNs) && (oldHold.type==Json::Null || UInt(oldHold)),"review_ns");
  Need(Same(reply.at("session_id"),request.at("session_id")) &&
       Same(reply.at("request_id"),request.at("request_id")),"reply_request_reference");
  Need(snapshot.at("observed_ns").integer<=reviewNs.integer,"future_reply_claim");
  if (Eq(reply.at("kind"),"ack")) {
    Need(Same(reply.at("for_kind"),kind),"reply_for_kind");
    if (!reply.at("declared_accepted").boolean)
      return Review("declared_refusal_not_execution",kind,false,backend);
  }
  const std::map<std::string,std::string> phases={{"reserve","reserved"},{"seal","sealed"},{"result_ref","consumed"},{"stop","tripped"}};
  Need(Eq(snapshot.at("phase"),phases.at(kind.string)),"reply_phase");
  if (!Eq(kind,"stop")) {
    const auto reserve=Context(reserveRaw);const auto seal=sealRaw?Decode(*sealRaw):Json{};
    Reference(request,reserve,seal,configured,frozen);
    Need(Same(snapshot.at("record"),request.at("record")),"reply_record_reference");
    Need(Same(snapshot.at("view_digests"),request.at("view_digests")),"reply_digest_reference");
    const auto wanted=Eq(kind,"reserve")?Json{}:Eq(kind,"seal")?request.at("request_id"):request.at("seal_request_id");
    Need(Same(snapshot.at("seal_request_id"),wanted),"reply_seal_reference");
    const auto &record=request.at("record");auto deadline=record.at("deadline_ns").integer;
    if (oldHold.type!=Json::Null && oldHold.integer<deadline) deadline=oldHold.integer;
    Need(record.at("available_ns").integer<=snapshot.at("observed_ns").integer &&
         reviewNs.integer<deadline,"reply_budget_exhausted");
  }
  return Review("synthetic_reply_claim_only",kind,true,backend);
}
}  // namespace toponav_pending_offline
