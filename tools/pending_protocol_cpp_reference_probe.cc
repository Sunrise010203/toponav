// Opt-in bounded binary envelope. Four raw packets are NEVER JSON re-encoded.
#include "pending_protocol_cpp_reference.hh"
#include <array>
#include <iostream>

int main(int argc,char **) {
  namespace d=toponav_pending_offline;
  if (argc!=1) { std::cerr<<"no arguments; offline TPQ1 envelope only\n";return 64; }
  constexpr std::size_t Limit=4+5*4+4096+4*(d::MaxBytes+1);
  std::array<char,Limit+1> buffer{};
  std::cin.read(buffer.data(),static_cast<std::streamsize>(buffer.size()));
  const std::string_view raw(buffer.data(),static_cast<std::size_t>(std::cin.gcount()));
  d::Json result;std::string error;
  try {
    d::Need(!std::cin.bad() && raw.size()<=Limit,"envelope_byte_bound");
    d::Need(raw.substr(0,4)=="TPQ1","envelope_magic");std::size_t pos=4;
    std::array<d::RawContext,5> fields{};
    for (std::size_t i=0;i<fields.size();++i) {
      d::Need(pos+4<=raw.size(),"envelope_length");std::uint32_t n=0;
      for (int k=0;k<4;++k) n=(n<<8)|static_cast<unsigned char>(raw[pos++]);
      if (n==UINT32_MAX) { d::Need(i>=2,"required_envelope_field");continue; }
      d::Need(n<=(i==0?4096:d::MaxBytes+1) && n<=raw.size()-pos,"envelope_field_bound");
      fields[i]=raw.substr(pos,n);pos+=n;
    }
    d::Need(pos==raw.size(),"envelope_trailing");
    const auto controls=d::Parser(*fields[0]).parse();
    d::Shape(controls,{"operation","required_views","views_frozen","backend","review_ns","old_hold_deadline_ns"},"controls_schema");
    d::Need(controls.at("views_frozen").type==d::Json::Boolean,"controls_frozen_bool");
    const auto &op=controls.at("operation");const bool frozen=controls.at("views_frozen").boolean;
    const auto &views=controls.at("required_views");d::Json review,repeated;
    if (d::Eq(op,"schema")) {
      const auto message=d::Decode(*fields[1]);review=d::Review("decoded_schema_not_admission",message.at("kind"),false);
    } else if (d::Eq(op,"reference") || d::Eq(op,"repeat")) {
      review=d::ReviewReference(*fields[1],fields[2],fields[3],views,frozen);
      if (d::Eq(op,"repeat")) repeated=d::ReviewReference(*fields[1],fields[2],fields[3],views,frozen);
    } else {
      d::Need(d::Eq(op,"reply") && fields[4].has_value(),"operation_or_reply");
      review=d::ReviewReply(*fields[1],*fields[4],fields[2],fields[3],views,frozen,controls.at("backend"),
                            controls.at("review_ns"),controls.at("old_hold_deadline_ns"));
    }
    result=d::Json::Obj();result.object={{"review",review},{"repeated_review",repeated}};
  } catch (const d::Refusal &refusal) { error=refusal.what(); }
  auto out=d::Review("offline_caller_snapshot_not_admission",d::Json{},false);
  out.object["consistent"]=d::Json::Bool(error.empty());out.object["result"]=result;
  out.object["error"]=error.empty()?d::Json{}:d::Json::Str(error);
  std::cout<<d::Dump(out)<<'\n';return error.empty()?0:2;
}
