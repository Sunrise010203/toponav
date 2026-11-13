// PURE bounded outer trace JSON parser; adapted from sealed 2P parser.
// Separate limits and Python-compatible escaped-surrogate strings, no IO/state.
#pragma once
#include "pending_protocol_cpp_reference.hh"

namespace toponav_pending_trace_offline {
using namespace toponav_pending_offline;
constexpr std::size_t TraceMaxBytes=262144,TraceMaxDepth=12,TraceMaxEvents=128;
class TraceParser {
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
          // Match Python JSON strings, retaining lone escaped surrogates
          // internally as CESU-8. Raw UTF-8 input still rejects surrogates.
          if (cp>=0xd800 && cp<=0xdbff && input.substr(pos,2)=="\\u" && pos+6<=input.size()) {
            unsigned low=0;bool valid=true;
            for (std::size_t k=pos+2;k<pos+6;++k) {
              const char h=input[k];unsigned d=16;
              if (h>='0' && h<='9') d=h-'0';
              else if (h>='a' && h<='f') d=h-'a'+10;
              else if (h>='A' && h<='F') d=h-'A'+10;
              if (d==16) valid=false;
              low=(low<<4)|(d&15);
            }
            if (valid && low>=0xdc00 && low<=0xdfff) { pos+=6;cp=0x10000+((cp-0xd800)<<10)+(low-0xdc00); }
          }
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
    Need(c=='{' || c=='[',"message_json");Need(depth<TraceMaxDepth,"json_depth_bound");++pos;
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
  explicit TraceParser(std::string_view raw):input(raw) {}
  Json parse() {
    Need(!input.empty() && input.size()<=TraceMaxBytes,"message_byte_bound");Utf8(input);
    std::size_t depth=0;bool quoted=false,escaped=false;
    for (const char c:input) {
      if (quoted) {
        if (escaped) escaped=false;else if (c=='\\') escaped=true;else if (c=='"') quoted=false;
      } else if (c=='"') quoted=true;
      else if (c=='[' || c=='{') { ++depth;Need(depth<=TraceMaxDepth,"json_depth_bound"); }
      else if (c==']' || c=='}') { if (depth) --depth; }
    }
    Json out=value(0);space();Need(pos==input.size(),"message_json");return out;
  }
};

inline bool LoneSurrogate(std::string_view s,std::size_t i) {
  return i+2<s.size() && static_cast<unsigned char>(s[i])==0xed &&
    static_cast<unsigned char>(s[i+1])>=0xa0 && static_cast<unsigned char>(s[i+1])<=0xbf &&
    (static_cast<unsigned char>(s[i+2])&0xc0)==0x80;
}
inline std::size_t Characters(std::string_view s) {
  std::size_t n=0;for (const unsigned char c:s) if ((c&0xc0)!=0x80) ++n;return n;
}
inline std::string TraceQuote(std::string_view s) {
  std::string out="\"";const char *hex="0123456789abcdef";
  for (std::size_t i=0;i<s.size();++i) {
    if (LoneSurrogate(s,i)) {
      const unsigned cp=((static_cast<unsigned char>(s[i])&15)<<12)|
        ((static_cast<unsigned char>(s[i+1])&63)<<6)|(static_cast<unsigned char>(s[i+2])&63);
      out+="\\u";for (int shift=12;shift>=0;shift-=4) out+=hex[(cp>>shift)&15];i+=2;
    } else {
      const auto c=static_cast<unsigned char>(s[i]);
      if (c=='"' || c=='\\') { out+='\\';out+=static_cast<char>(c); }
      else if (c<32) { out+="\\u00";out+=hex[c>>4];out+=hex[c&15]; }
      else out+=static_cast<char>(c);
    }
  }
  return out+'"';
}
inline std::string TraceDump(const Json &j) {
  if (j.type==Json::String) return TraceQuote(j.string);
  if (j.type!=Json::Array && j.type!=Json::Object) return Dump(j);
  std::string out=j.type==Json::Array?"[":"{";bool first=true;
  if (j.type==Json::Array) for (const auto &v:j.array) { if (!first) out+=',';first=false;out+=TraceDump(v); }
  else for (const auto &p:j.object) { if (!first) out+=',';first=false;out+=TraceQuote(p.first)+':'+TraceDump(p.second); }
  return out+(j.type==Json::Array?']':'}');
}
}  // namespace toponav_pending_trace_offline
