// Standalone OFFLINE schema draft. No production/Gazebo header, owner or IO.
// Deliberately implements only 2N decode_message, NOT reference/reply admission.
#pragma once
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace toponav_pending_offline {
constexpr std::size_t MaxBytes=8192, MaxDepth=8;
constexpr std::int64_t MaxNs=INT64_MAX, MaxId=INT32_MAX, LeaseNs=800000000;
struct Refusal : std::runtime_error { using std::runtime_error::runtime_error; };
inline void Need(bool ok,const char *code) { if (!ok) throw Refusal(code); }
struct Json {
  enum Type { Null, Boolean, Integer, String, Array, Object } type=Null;
  bool boolean=false;
  std::int64_t integer=0;
  std::string string;
  std::vector<Json> array;
  std::map<std::string,Json> object;
  static Json Bool(bool b) { Json j;j.type=Boolean;j.boolean=b;return j; }
  static Json Int(std::int64_t n) { Json j;j.type=Integer;j.integer=n;return j; }
  static Json Str(std::string s) { Json j;j.type=String;j.string=std::move(s);return j; }
  static Json Arr() { Json j;j.type=Array;return j; }
  static Json Obj() { Json j;j.type=Object;return j; }
  const Json &at(const std::string &key) const { return object.at(key); }
};

inline void Utf8(std::string_view raw) {
  for (std::size_t i=0;i<raw.size();) {
    const auto c=static_cast<unsigned char>(raw[i++]);
    if (c<128) continue;
    unsigned n=0;std::uint32_t cp=0,minimum=0;
    if (c>=0xc2 && c<=0xdf) { n=1;cp=c&31;minimum=0x80; }
    else if (c>=0xe0 && c<=0xef) { n=2;cp=c&15;minimum=0x800; }
    else if (c>=0xf0 && c<=0xf4) { n=3;cp=c&7;minimum=0x10000; }
    else throw Refusal("message_utf8");
    Need(i+n<=raw.size(),"message_utf8");
    while (n--) {
      const auto x=static_cast<unsigned char>(raw[i++]);
      Need((x&0xc0)==0x80,"message_utf8");cp=(cp<<6)|(x&63);
    }
    Need(cp>=minimum && cp<=0x10ffff && !(cp>=0xd800 && cp<=0xdfff),"message_utf8");
  }
}

class Parser {
  std::string_view input;std::size_t pos=0;
  char peek() const { return pos<input.size()?input[pos]:'\0'; }
  char take() { Need(pos<input.size(),"message_json");return input[pos++]; }
  void space() { while (peek()==' ' || peek()=='\n' || peek()=='\r' || peek()=='\t') ++pos; }
  void literal(std::string_view word) {
    Need(input.substr(pos,word.size())==word,"message_json");pos+=word.size();
  }
  unsigned hex4() {
    unsigned cp=0;
    for (int i=0;i<4;++i) {
      const char c=take();unsigned d=16;
      if (c>='0' && c<='9') d=c-'0';
      else if (c>='a' && c<='f') d=c-'a'+10;
      else if (c>='A' && c<='F') d=c-'A'+10;
      Need(d<16,"message_json");cp=(cp<<4)|d;
    }
    return cp;
  }
  static void append(std::string &out,unsigned cp) {
    if (cp<0x80) out.push_back(static_cast<char>(cp));
    else if (cp<0x800) {
      out.push_back(static_cast<char>(0xc0|(cp>>6)));out.push_back(static_cast<char>(0x80|(cp&63)));
    } else if (cp<0x10000) {
      out.push_back(static_cast<char>(0xe0|(cp>>12)));out.push_back(static_cast<char>(0x80|((cp>>6)&63)));
      out.push_back(static_cast<char>(0x80|(cp&63)));
    } else {
      out.push_back(static_cast<char>(0xf0|(cp>>18)));out.push_back(static_cast<char>(0x80|((cp>>12)&63)));
      out.push_back(static_cast<char>(0x80|((cp>>6)&63)));out.push_back(static_cast<char>(0x80|(cp&63)));
    }
  }
  std::string text() {
    Need(take()=='"',"message_json");std::string out;
    while (true) {
      const auto c=static_cast<unsigned char>(take());
      if (c=='"') return out;
      Need(c>=32,"message_json");
      if (c!='\\') { out.push_back(static_cast<char>(c));continue; }
      const char e=take();
      switch (e) {
        case '"':case '\\':case '/':out.push_back(e);break;
        case 'b':out.push_back('\b');break;case 'f':out.push_back('\f');break;
        case 'n':out.push_back('\n');break;case 'r':out.push_back('\r');break;case 't':out.push_back('\t');break;
        case 'u': {
          unsigned cp=hex4();
          if (cp>=0xd800 && cp<=0xdbff) {
            Need(take()=='\\' && take()=='u',"unicode_surrogate");const unsigned low=hex4();
            Need(low>=0xdc00 && low<=0xdfff,"unicode_surrogate");cp=0x10000+((cp-0xd800)<<10)+(low-0xdc00);
          } else Need(!(cp>=0xdc00 && cp<=0xdfff),"unicode_surrogate");
          append(out,cp);break;
        }
        default:throw Refusal("message_json");
      }
    }
  }
  Json number() {
    const bool negative=peek()=='-';if (negative) ++pos;
    Need(peek()>='0' && peek()<='9',"integer_json_only");
    std::uint64_t value=0;const auto start=pos;
    while (peek()>='0' && peek()<='9') {
      const unsigned digit=take()-'0';
      Need(value<=(static_cast<std::uint64_t>(MaxNs)-digit)/10,"integer_bound");value=value*10+digit;
    }
    Need(pos-start==1 || input[start]!='0',"message_json");
    Need(peek()!='.' && peek()!='e' && peek()!='E',"integer_json_only");
    // Values below -INT64_MAX cannot be valid in the closed unsigned schema.
    return Json::Int(negative?-static_cast<std::int64_t>(value):static_cast<std::int64_t>(value));
  }
  Json value(std::size_t depth) {
    space();const char c=peek();
    if (c=='"') return Json::Str(text());
    if (c=='n') { literal("null");return {}; }
    if (c=='t') { literal("true");return Json::Bool(true); }
    if (c=='f') { literal("false");return Json::Bool(false); }
    if (c=='-' || (c>='0' && c<='9')) return number();
    Need(c=='{' || c=='[',"message_json");Need(depth<MaxDepth,"json_depth_bound");++pos;
    Json out=c=='{'?Json::Obj():Json::Arr();const char end=c=='{'?'}':']';space();
    if (peek()==end) { ++pos;return out; }
    while (true) {
      space();
      if (c=='{') {
        Need(out.object.size()<16,"object_field_bound");const auto key=text();
        // All allowed keys are ASCII; a byte bound on a non-ASCII unknown key
        // may reject earlier, never expand this closed message grammar.
        Need(key.size()<=64,"object_key_bound");Need(out.object.count(key)==0,"duplicate_field");
        space();Need(take()==':',"message_json");out.object.emplace(key,value(depth+1));
      } else out.array.push_back(value(depth+1));
      space();const char next=take();if (next==end) return out;Need(next==',',"message_json");
    }
  }
 public:
  explicit Parser(std::string_view raw):input(raw) {}
  Json parse() {
    Need(!input.empty() && input.size()<=MaxBytes,"message_byte_bound");Utf8(input);
    std::size_t depth=0;bool quoted=false,escaped=false;
    for (const char c:input) {
      if (quoted) {
        if (escaped) escaped=false;else if (c=='\\') escaped=true;else if (c=='"') quoted=false;
      } else if (c=='"') quoted=true;
      else if (c=='[' || c=='{') { ++depth;Need(depth<=MaxDepth,"json_depth_bound"); }
      else if (c==']' || c=='}') { if (depth) --depth; }
    }
    Json out=value(0);space();Need(pos==input.size(),"message_json");return out;
  }
};

inline bool Eq(const Json &j,std::string_view s) { return j.type==Json::String && j.string==s; }
inline bool UInt(const Json &j,std::int64_t max=MaxNs) { return j.type==Json::Integer && j.integer>=0 && j.integer<=max; }
inline bool Identity(const Json &j) {
  if (j.type!=Json::String || j.string.empty() || j.string.size()>128) return false;
  for (const unsigned char c:j.string)
    if (!((c>='a' && c<='z') || (c>='A' && c<='Z') || (c>='0' && c<='9') ||
        c=='-' || c=='_' || c=='/' || c=='.' || c==':')) return false;
  return true;
}
inline void Shape(const Json &j,std::initializer_list<const char *> keys,const char *code) {
  Need(j.type==Json::Object && j.object.size()==keys.size(),code);
  for (const auto key:keys) Need(j.object.count(key)==1,code);
}
inline void Views(const Json &j) {
  Need(j.type==Json::Array && j.array.size()>=4 && j.array.size()<=6,"view_schema");
  const std::vector<std::string> base={"front","left","right","back"};
  for (std::size_t i=0;i<4;++i) Need(Eq(j.array[i],base[i]),"view_order_or_duplicate");
  if (j.array.size()==5) Need(Eq(j.array[4],"up") || Eq(j.array[4],"down"),"view_order_or_duplicate");
  if (j.array.size()==6) Need(Eq(j.array[4],"up") && Eq(j.array[5],"down"),"view_order_or_duplicate");
}
inline void Record(const Json &j) {
  Shape(j,{"clock_domain","host_boot_id","generation","frame_id","captured_sim_ns","required_views","available_ns","deadline_ns"},"record_schema");
  Need(Eq(j.at("clock_domain"),"synthetic_monotonic_ns") && Identity(j.at("host_boot_id")),"synthetic_clock_identity");
  Need(UInt(j.at("generation"),MaxId) && j.at("generation").integer>0 && Identity(j.at("frame_id")),"record_identity");
  Need(UInt(j.at("captured_sim_ns")) && UInt(j.at("available_ns"),MaxNs-LeaseNs) && UInt(j.at("deadline_ns")),"record_ns");
  Need(j.at("deadline_ns").integer==j.at("available_ns").integer+LeaseNs,"original_work_budget");Views(j.at("required_views"));
}
inline void Digests(const Json &j,const Json &record) {
  const auto &views=record.at("required_views").array;
  Need(j.type==Json::Array && j.array.size()==views.size(),"digest_view_count");
  for (std::size_t i=0;i<views.size();++i) {
    const auto &pair=j.array[i];Need(pair.type==Json::Array && pair.array.size()==2 && Eq(pair.array[0],views[i].string),"digest_view_order");
    const auto &hash=pair.array[1];Need(hash.type==Json::String && hash.string.size()==64,"digest_schema");
    Need(hash.string.find_first_not_of("0123456789abcdef")==std::string::npos,"digest_schema");
  }
}
inline void Snapshot(const Json &j) {
  Shape(j,{"backend","phase","record","view_digests","seal_request_id","observed_ns"},"snapshot_schema");
  Need(Eq(j.at("backend"),"python") || Eq(j.at("backend"),"native"),"backend_schema");
  const auto &phase=j.at("phase");
  Need(Eq(phase,"disarmed") || Eq(phase,"reserved") || Eq(phase,"sealed") || Eq(phase,"consumed") || Eq(phase,"tripped"),"phase_schema");
  Need(UInt(j.at("observed_ns")),"snapshot_ns");const auto &record=j.at("record"),&seal=j.at("seal_request_id");
  const bool hasRecord=record.type!=Json::Null,sealed=seal.type!=Json::Null;
  if (hasRecord) Record(record);
  Need(!sealed || (UInt(seal,MaxId) && seal.integer>0),"seal_reference_schema");
  if (Eq(phase,"disarmed")) Need(!hasRecord && !sealed,"disarmed_has_no_record");
  else if (!Eq(phase,"tripped")) Need(hasRecord && sealed!=Eq(phase,"reserved"),"phase_record_binding");
  else Need(hasRecord || !sealed,"tripped_record_binding");
  if (sealed) { Need(hasRecord,"sealed_record_required");Digests(j.at("view_digests"),record); }
  else Need(j.at("view_digests").type==Json::Array && j.at("view_digests").array.empty(),"unsealed_no_digests");
}
inline Json Decode(std::string_view raw) {
  const Json j=Parser(raw).parse();Need(j.type==Json::Object && j.object.count("kind") && j.at("kind").type==Json::String,"message_object_kind");
  const auto &kind=j.at("kind").string;
  if (kind=="reserve") Shape(j,{"protocol","scope","kind","session_id","request_id","capabilities","record"},"message_fields");
  else if (kind=="seal") Shape(j,{"protocol","scope","kind","session_id","request_id","capabilities","record","view_digests"},"message_fields");
  else if (kind=="result_ref") Shape(j,{"protocol","scope","kind","session_id","request_id","capabilities","record","view_digests","seal_request_id"},"message_fields");
  else if (kind=="ack") Shape(j,{"protocol","scope","kind","session_id","request_id","capabilities","for_kind","accepted","status"},"message_fields");
  else if (kind=="status") Shape(j,{"protocol","scope","kind","session_id","request_id","capabilities","status"},"message_fields");
  else if (kind=="stop") Shape(j,{"protocol","scope","kind","session_id","request_id","reason"},"message_fields");
  else throw Refusal("message_kind");
  Need(Eq(j.at("protocol"),"toponav_pending_offline_draft_v1") && Eq(j.at("scope"),"offline_schema_only"),"offline_protocol_only");
  Need(Identity(j.at("session_id")) && UInt(j.at("request_id"),MaxId) && j.at("request_id").integer>0,"message_identity");
  Json out=Json::Obj();out.object={{"kind",j.at("kind")},{"session_id",j.at("session_id")},{"request_id",j.at("request_id")},
    {"record",{}},{"view_digests",Json::Arr()},{"seal_request_id",{}},{"for_kind",{}},{"declared_accepted",{}},{"snapshot",{}}};
  if (kind=="stop") {
    const auto &r=j.at("reason");Need(Eq(r,"cancel") || Eq(r,"terminal") || Eq(r,"failure") || Eq(r,"shutdown"),"stop_reason");return out;
  }
  const auto &caps=j.at("capabilities");const std::vector<std::string> expected={"reserve","seal","result_ref","pending_status"};
  Need(caps.type==Json::Array && caps.array.size()==4,"no_capability_downgrade");
  for (std::size_t i=0;i<4;++i) Need(Eq(caps.array[i],expected[i]),"no_capability_downgrade");
  if (kind=="reserve" || kind=="seal" || kind=="result_ref") {
    Record(j.at("record"));out.object["record"]=j.at("record");
    if (kind!="reserve") { Digests(j.at("view_digests"),j.at("record"));out.object["view_digests"]=j.at("view_digests"); }
    if (kind=="result_ref") {
      Need(UInt(j.at("seal_request_id"),MaxId) && j.at("seal_request_id").integer>0,"seal_reference_schema");out.object["seal_request_id"]=j.at("seal_request_id");
    }
    return out;
  }
  Snapshot(j.at("status"));out.object["snapshot"]=j.at("status");if (kind=="status") return out;
  const auto &forKind=j.at("for_kind");
  Need(Eq(forKind,"reserve") || Eq(forKind,"seal") || Eq(forKind,"result_ref") || Eq(forKind,"stop"),"ack_kind");
  Need(j.at("accepted").type==Json::Boolean,"ack_accepted_bool");
  const std::map<std::string,std::string> phases={{"reserve","reserved"},{"seal","sealed"},{"result_ref","consumed"},{"stop","tripped"}};
  Need(Eq(j.at("status").at("phase"),j.at("accepted").boolean?phases.at(forKind.string):"tripped"),"ack_phase");
  out.object["for_kind"]=forKind;out.object["declared_accepted"]=j.at("accepted");return out;
}

inline std::string Quote(std::string_view s) {
  std::string out="\"";const char *hex="0123456789abcdef";
  for (const unsigned char c:s) {
    if (c=='"' || c=='\\') { out+='\\';out+=static_cast<char>(c); }
    else if (c<32) { out+="\\u00";out+=hex[c>>4];out+=hex[c&15]; }
    else out+=static_cast<char>(c);
  }
  return out+'"';
}
inline std::string Dump(const Json &j) {
  if (j.type==Json::Null) return "null";
  if (j.type==Json::Boolean) return j.boolean?"true":"false";
  if (j.type==Json::Integer) return std::to_string(j.integer);
  if (j.type==Json::String) return Quote(j.string);
  std::string out=j.type==Json::Array?"[":"{";bool first=true;
  if (j.type==Json::Array) for (const auto &value:j.array) { if (!first) out+=',';first=false;out+=Dump(value); }
  else for (const auto &pair:j.object) { if (!first) out+=',';first=false;out+=Quote(pair.first)+':'+Dump(pair.second); }
  return out+(j.type==Json::Array?']':'}');
}
}  // namespace toponav_pending_offline
