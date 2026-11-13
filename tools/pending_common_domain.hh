// PURE opt-in TPT1 contract. Original 2P/2Q namespaces and files are unchanged.
#pragma once
#include "pending_protocol_cpp_reference.hh"
#include <array>

namespace toponav_pending_common_v1 {
namespace d=toponav_pending_offline;
constexpr std::size_t Limit=4+5*4+4096+4*(d::MaxBytes+1);
constexpr const char *Profile="toponav_pending_common_domain_v1";
struct Input { d::Json controls;std::array<d::RawContext,4> packets; };
inline Input Preflight(std::string_view raw) {
  d::Need(raw.size()<=Limit,"domain_byte_bound");d::Need(raw.substr(0,4)=="TPT1","domain_magic");
  std::size_t pos=4;std::array<d::RawContext,5> fields{};
  for (std::size_t i=0;i<fields.size();++i) {
    d::Need(pos+4<=raw.size(),"domain_length");std::uint32_t n=0;
    for (int k=0;k<4;++k) n=(n<<8)|static_cast<unsigned char>(raw[pos++]);
    if (n==UINT32_MAX) { d::Need(i>=2,"domain_required_field");continue; }
    d::Need(n<=(i==0?4096:d::MaxBytes+1) && n<=raw.size()-pos,"domain_field_bound");
    fields[i]=raw.substr(pos,n);pos+=n;
  }
  d::Need(pos==raw.size(),"domain_trailing");d::Json c;
  try {
    for (const auto ch:*fields[0]) d::Need(static_cast<unsigned char>(ch)<128,"domain_controls");
    c=d::Parser(*fields[0]).parse();
    d::Shape(c,{"operation","required_views","views_frozen","backend","review_ns","old_hold_deadline_ns"},"domain_controls");
    const auto &op=c.at("operation");
    d::Need(d::Eq(op,"schema") || d::Eq(op,"reference") || d::Eq(op,"repeat") || d::Eq(op,"reply"),"domain_controls");
    d::Need(c.at("views_frozen").type==d::Json::Boolean && c.at("views_frozen").boolean,"domain_controls");
    d::Views(c.at("required_views"));
    if (d::Eq(op,"reply")) {
      d::Need(d::Eq(c.at("backend"),"python") || d::Eq(c.at("backend"),"native"),"domain_controls");
      d::Need(d::UInt(c.at("review_ns")),"domain_controls");
      d::Need(c.at("old_hold_deadline_ns").type==d::Json::Null || d::UInt(c.at("old_hold_deadline_ns")),"domain_controls");
    } else for (const auto key:{"backend","review_ns","old_hold_deadline_ns"})
      d::Need(c.at(key).type==d::Json::Null,"domain_controls");
    d::Need(d::Dump(c)==*fields[0],"domain_controls");
  } catch (const d::Refusal &) { throw d::Refusal("domain_controls"); }
  d::Need(!d::Eq(c.at("operation"),"reply") || fields[4].has_value(),"domain_reply_required");
  return {c,{fields[1],fields[2],fields[3],fields[4]}};
}
inline d::Json ReviewEnvelope(std::string_view raw) {
  auto out=d::Review("offline_common_domain_not_admission",d::Json{},false);
  // The domain contract has its own shape, not a normalized old review result.
  out.object.erase("message_kind");out.object.erase("synthetic_reference_matches");out.object.erase("backend_claim");
  out.object["profile"]=d::Json::Str(Profile);out.object["domain_accepted"]=d::Json::Bool(false);
  out.object["stage"]=d::Json::Str("domain_refused");out.object["consistent"]=d::Json::Bool(false);
  out.object["result"]={};out.object["error"]={};
  for (const auto key:{"runtime_one_time_consumption_enforced","cross_run_replay_prevented"}) out.object[key]=d::Json::Bool(false);
  Input input;
  try { input=Preflight(raw); }
  catch (const d::Refusal &e) { out.object["error"]=d::Json::Str(e.what());return out; }
  out.object["domain_accepted"]=d::Json::Bool(true);out.object["stage"]=d::Json::Str("review_refused");
  try {
    const auto &c=input.controls;const auto &p=input.packets;const auto &op=c.at("operation");d::Json review,repeated;
    if (d::Eq(op,"schema")) review=d::Review("decoded_schema_not_admission",d::Decode(*p[0]).at("kind"),false);
    else if (d::Eq(op,"reply")) review=d::ReviewReply(*p[0],*p[3],p[1],p[2],c.at("required_views"),true,
        c.at("backend"),c.at("review_ns"),c.at("old_hold_deadline_ns"));
    else {
      review=d::ReviewReference(*p[0],p[1],p[2],c.at("required_views"),true);
      if (d::Eq(op,"repeat")) repeated=d::ReviewReference(*p[0],p[1],p[2],c.at("required_views"),true);
    }
    auto result=d::Json::Obj();result.object={{"review",review},{"repeated_review",repeated}};
    out.object["result"]=result;out.object["stage"]=d::Json::Str("review_returned");out.object["consistent"]=d::Json::Bool(true);
  } catch (const d::Refusal &e) { out.object["error"]=d::Json::Str(e.what()); }
  return out;
}
} // namespace toponav_pending_common_v1
