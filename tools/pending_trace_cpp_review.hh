// PURE offline transcript lint. Local order variables are NOT a runtime owner.
#pragma once
#include "pending_trace_cpp_json.hh"

namespace toponav_pending_trace_offline {
inline bool Backend(const Json &j) { return Eq(j,"python") || Eq(j,"native"); }
inline bool WireAction(const std::string &a) {
  return a=="freeze" || a=="hash_done" || a=="result" || a=="send" || a=="reply" || a=="late_reply";
}
inline Json TraceSchema(std::string_view raw) {
  const auto doc=TraceParser(raw).parse();
  Shape(doc,{"format","scope","session_id","required_views","old_hold_deadline_ns","events"},"trace_fields");
  Need(Eq(doc.at("format"),"pending_trace_claims_v1") && Eq(doc.at("scope"),"offline_transcript_only") && Identity(doc.at("session_id")),"trace_namespace");
  Views(doc.at("required_views"));const auto &hold=doc.at("old_hold_deadline_ns");
  Need(hold.type==Json::Null || UInt(hold),"trace_old_hold");const auto &events=doc.at("events");
  Need(events.type==Json::Array && !events.array.empty() && events.array.size()<=TraceMaxEvents,"trace_event_bound");
  for (const auto &e:events.array) {
    Need(e.type==Json::Object && e.object.count("action") && e.at("action").type==Json::String,"event_action");
    const auto &a=e.at("action").string;
    if (WireAction(a)) Shape(e,{"action","actor","target","at_ns","wire"},"event_fields");
    else if (a=="failure" || a=="diagnostic") Shape(e,{"action","actor","target","at_ns","error"},"event_fields");
    else if (a=="ack_unknown") Shape(e,{"action","actor","target","at_ns","request_id","backend","error"},"event_fields");
    else if (a=="finish") Shape(e,{"action","actor","target","at_ns","outcome"},"event_fields");
    else {
      Need(a=="hash_begin" || a=="save_begin" || a=="save_done" || a=="cancel" || a=="compute_begin" || a=="compute_done","event_action");
      Shape(e,{"action","actor","target","at_ns"},"event_fields");
    }
    Need(e.at("actor").type==Json::String && e.at("target").type==Json::String && UInt(e.at("at_ns")),"event_identity_time");
    if (WireAction(a)) Need(e.at("wire").type==Json::String && Characters(e.at("wire").string)<=MaxBytes,"event_wire");
    if (e.object.count("error")) {
      const auto &error=e.at("error");Need(error.type==Json::String && !error.string.empty() && error.string.size()<=256,"event_error");
      for (const unsigned char c:error.string) Need(c>=32 && c<=126,"event_error");
    }
    if (a=="ack_unknown") Need(UInt(e.at("request_id"),MaxId) && e.at("request_id").integer>0 && Backend(e.at("backend")),"unknown_ack_identity");
    if (a=="finish") Need(Eq(e.at("outcome"),"completed_claim") || Eq(e.at("outcome"),"failed_claim") || Eq(e.at("outcome"),"stopped_claim"),"finish_outcome");
  }
  return doc;
}
struct TraceSubmission {
  Json requestId,kind,backend;std::size_t sendIndex;std::int64_t sendNs;
  std::string outcome="unconfirmed";Json replies=Json::Arr();
};
inline Json ReviewTrace(std::string_view raw) {
  const auto doc=TraceSchema(raw);const auto &events=doc.at("events").array;
  const auto &configured=doc.at("required_views");const auto &hold=doc.at("old_hold_deadline_ns");
  std::optional<std::string> reserve,seal,result;
  std::map<std::int64_t,std::pair<Json,std::string>> requests;std::vector<TraceSubmission> sent;
  Json violations=Json::Arr(),failures=Json::Arr();
  bool fault=false,stopping=false,finished=false,cancelled=false,diagnosed=false;
  bool hashStarted=false,saveStarted=false,saved=false,computeStarted=false,computed=false;
  std::int64_t previous=-1;std::string disposition="incomplete_claim";
  auto append=[](Json &to,std::size_t index,const std::string &error) {
    Json row=Json::Arr();row.array={Json::Int(static_cast<std::int64_t>(index)),Json::Str(error)};to.array.push_back(row);
  };
  auto both=[&](const char *kind) {
    for (const auto b:{"python","native"}) {
      bool found=false;for (const auto &s:sent) if (Eq(s.kind,kind) && Eq(s.backend,b) && s.outcome=="declared_confirmed") found=true;
      if (!found) return false;
    }
    return true;
  };
  auto stopSent=[&]() {
    for (const auto b:{"python","native"}) {
      bool found=false;for (const auto &s:sent) if (Eq(s.kind,"stop") && Eq(s.backend,b)) found=true;
      if (!found) return false;
    }
    return true;
  };
  auto registerRequest=[&](const Json &message,const std::string &wire) {
    const auto id=message.at("request_id").integer;
    Need(requests.count(id)==0 && (requests.empty() || id>requests.rbegin()->first),"request_reuse_or_regression");
    requests.emplace(id,std::make_pair(message,wire));
  };
  auto budget=[&](std::int64_t now,bool checkHold=true) {
    Need(reserve.has_value(),"origin_missing");const auto r=Decode(*reserve).at("record");auto deadline=r.at("deadline_ns").integer;
    if (checkHold && hold.type!=Json::Null && hold.integer<deadline) deadline=hold.integer;
    Need(r.at("available_ns").integer<=now && now<deadline,"trace_budget_exhausted");
  };
  auto context=[](const std::optional<std::string> &value)->RawContext {
    return value?RawContext{*value}:std::nullopt;
  };
  for (std::size_t index=0;index<events.size();++index) {
    const auto &e=events[index];const auto &action=e.at("action").string;const auto now=e.at("at_ns").integer;
    Json message;std::string wire;std::optional<std::size_t> submission;
    try {
      // Preserve every decodable send BEFORE checking direction/session/order.
      if (WireAction(action)) {
        wire=e.at("wire").string;
        for (std::size_t k=0;k<wire.size();++k) Need(!LoneSurrogate(wire,k),"event_wire_utf8");
        message=Decode(wire);
        if (action=="send") {
          submission=sent.size();sent.push_back({message.at("request_id"),message.at("kind"),e.at("target"),index,now,"unconfirmed",Json::Arr()});
        }
        Need(Same(message.at("session_id"),doc.at("session_id")),"trace_session");
      }
      Need(now>=previous,"trace_time_regression");previous=now;Need(!finished,"event_after_finish");
      std::string actor="owner",target="local";
      if (action=="send") { Need(Backend(e.at("target")),"send_backend");target=e.at("target").string; }
      else if (action=="reply" || action=="late_reply") { Need(Backend(e.at("actor")),"reply_actor");actor=e.at("actor").string;target="owner"; }
      else if (action=="freeze") { actor="capture";target="owner"; }
      else if (action=="result" || action=="compute_done") { actor="controller";target="owner"; }
      else if (action=="compute_begin") target="controller";
      Need(Eq(e.at("actor"),actor) && Eq(e.at("target"),target),"event_direction");
      Need(!(fault || stopping) || action=="send" || action=="reply" || action=="late_reply" || action=="ack_unknown" ||
           action=="failure" || action=="cancel" || action=="diagnostic" || action=="finish","work_after_failure_or_stop");
      if (action=="freeze") {
        Need(!reserve && Eq(message.at("kind"),"reserve"),"single_origin_only");ReviewReference(wire,wire,std::nullopt,configured,true);
        Need(now==message.at("record").at("available_ns").integer,"origin_claim_time");registerRequest(message,wire);reserve=wire;budget(now);
      } else if (action=="hash_begin") {
        budget(now);Need(both("reserve") && !hashStarted,"reserve_barrier_before_hash");hashStarted=true;
      } else if (action=="hash_done") {
        budget(now);Need(hashStarted && !seal && Eq(message.at("kind"),"seal"),"hash_done_stage");
        ReviewReference(wire,context(reserve),std::nullopt,configured,true);registerRequest(message,wire);seal=wire;
      } else if (action=="save_begin") {
        budget(now);Need(both("seal") && !saveStarted,"seal_barrier_before_save");saveStarted=true;
      } else if (action=="save_done") {
        budget(now);Need(saveStarted && !saved,"save_done_stage");saved=true;
      } else if (action=="compute_begin") {
        budget(now);Need(both("seal") && saved && !computeStarted,"seal_save_barrier_before_compute");computeStarted=true;
      } else if (action=="compute_done") {
        budget(now);Need(computeStarted && !computed,"compute_done_stage");computed=true;
      } else if (action=="result") {
        budget(now);Need(computed && !result && Eq(message.at("kind"),"result_ref"),"result_stage_or_duplicate");
        ReviewReference(wire,context(reserve),context(seal),configured,true);registerRequest(message,wire);result=wire;
      } else if (action=="send") {
        if (Eq(message.at("kind"),"stop")) {
          if (!stopping) registerRequest(message,wire);
          stopping=true;
        } else {
          Need(!fault && !stopping,"nonstop_after_failure_or_stop");budget(now);
          const auto *wanted=Eq(message.at("kind"),"reserve")?&reserve:Eq(message.at("kind"),"seal")?&seal:Eq(message.at("kind"),"result_ref")?&result:nullptr;
          Need(wanted && *wanted && **wanted==wire,"send_unprepared_or_changed");
        }
        const auto id=message.at("request_id").integer;
        Need(requests.count(id) && requests.at(id).second==wire,"send_request_reference");
        for (std::size_t i=0;i<sent.size();++i) if (i!=*submission)
          Need(!Same(sent[i].requestId,message.at("request_id")) || !Same(sent[i].backend,e.at("target")),"duplicate_send_no_retry");
      } else if (action=="reply" || action=="late_reply" || action=="ack_unknown") {
        const auto backend=action=="ack_unknown"?e.at("backend"):e.at("actor");
        const auto id=(action=="ack_unknown"?e.at("request_id"):message.at("request_id")).integer;
        std::vector<std::size_t> matches;
        for (std::size_t i=0;i<sent.size();++i) if (sent[i].requestId.integer==id && Same(sent[i].backend,backend)) matches.push_back(i);
        Need(matches.size()==1 && requests.count(id),"unsolicited_or_ambiguous_reply");
        auto &entry=sent[matches[0]];const auto &request=requests.at(id).first;const auto &requestWire=requests.at(id).second;
        if (action=="late_reply") {
          Need(fault && entry.outcome!="declared_confirmed","late_reply_requires_uncertainty");
          Need((Eq(message.at("kind"),"ack") || Eq(message.at("kind"),"status")) && Same(message.at("snapshot").at("backend"),backend),"late_reply_backend_kind");
          if (Eq(message.at("kind"),"ack")) Need(Same(message.at("for_kind"),request.at("kind")),"late_reply_for_kind");
          entry.replies.array.push_back(Json::Int(static_cast<std::int64_t>(index))); // NEVER upgrade uncertainty.
        } else if (action=="ack_unknown") {
          Need(entry.outcome=="unconfirmed","unknown_ack_stage");entry.outcome="unknown_outcome";fault=true;append(failures,index,e.at("error").string);
        } else {
          Need(entry.outcome=="unconfirmed","duplicate_or_post_unknown_reply");
          Need((!fault && !stopping) || Eq(request.at("kind"),"stop"),"reply_after_failure");
          const auto reviewed=ReviewReply(requestWire,wire,context(reserve),context(seal),configured,true,backend,Json::Int(now),hold);
          Need(message.at("snapshot").at("observed_ns").integer>=entry.sendNs,"reply_before_send");
          entry.replies.array.push_back(Json::Int(static_cast<std::int64_t>(index)));
          if (Eq(reviewed.at("classification"),"declared_refusal_not_execution")) {
            entry.outcome="declared_refusal";fault=true;append(failures,index,"declared_refusal");
          } else entry.outcome="declared_confirmed";
        }
      } else if (action=="failure") { fault=true;append(failures,index,e.at("error").string); }
      else if (action=="cancel") { Need(stopSent() && !cancelled,"stop_before_cancel");cancelled=true; }
      else if (action=="diagnostic") {
        Need(stopSent() && cancelled && fault && !diagnosed,"stop_cancel_before_diagnostic");
        bool found=false;for (const auto &v:failures.array) if (Same(v.array[1],e.at("error"))) found=true;
        Need(found,"diagnostic_preserves_original_error");diagnosed=true;
      } else if (action=="finish") {
        const auto &outcome=e.at("outcome");
        if (Eq(outcome,"completed_claim")) {
          Need(!fault && !stopping && both("result_ref"),"completed_requires_both_result_claims");budget(now,false);
        } else {
          Need(stopSent() && cancelled,"finish_requires_stop_and_cancel");
          Need(Eq(outcome,"failed_claim")?(fault && diagnosed):!fault,"finish_failure_classification");
        }
        disposition=outcome.string;finished=true;
      }
    } catch (const Refusal &error) { append(violations,index,error.what());fault=true; }
  }
  if (!finished) append(violations,events.size(),"missing_finish");
  if (!violations.array.empty()) disposition="rejected_transcript";
  Json rows=Json::Arr();
  for (const auto &s:sent) {
    Json row=Json::Obj();row.object={{"request_id",s.requestId},{"kind",s.kind},{"backend",s.backend},
      {"send_index",Json::Int(static_cast<std::int64_t>(s.sendIndex))},{"send_ns",Json::Int(s.sendNs)},
      {"outcome",Json::Str(s.outcome)},{"reply_indices",s.replies}};rows.array.push_back(row);
  }
  auto out=Review("caller_transcript_order_only",Json{},false);
  out.object.erase("message_kind");out.object.erase("synthetic_reference_matches");out.object.erase("backend_claim");
  out.object["consistent"]=Json::Bool(violations.array.empty());out.object["disposition"]=Json::Str(disposition);
  out.object["event_count"]=Json::Int(static_cast<std::int64_t>(events.size()));out.object["violations"]=violations;
  out.object["failures"]=failures;out.object["submissions"]=rows;
  out.object["runtime_one_time_consumption_enforced"]=Json::Bool(false);out.object["cross_run_replay_prevented"]=Json::Bool(false);
  return out;
}
}  // namespace toponav_pending_trace_offline
