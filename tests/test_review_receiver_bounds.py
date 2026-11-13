"""Compiled production decoder; instrumented views measure work, NOT braking."""
import hashlib
import json
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
HARNESS = r'''
#include <cstdlib>
#include <iostream>
#include <new>
#include <string>
#include <vector>
#include <algorithm>
static bool measuring=false;
static std::size_t allocations=0, allocated_bytes=0, key_reads=0, value_reads=0, visits=0;
void *operator new(std::size_t n) {
  if (measuring) {++allocations;allocated_bytes+=n;}
  if (void *p=std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void *p) noexcept {std::free(p);}
void operator delete(void *p, std::size_t) noexcept {std::free(p);}
struct Entry {
  std::string k;std::vector<std::string> values;
  const std::string &key() const {++key_reads;return k;}
  int value_size() const {return values.size();}
  const std::string &value(int n) const {++value_reads;return values.at(n);}
};
struct Header {
  std::vector<Entry> entries;
  int data_size() const {return entries.size();}
  const auto &data() const {return entries;}
  const auto &data(int n) const {++visits;return entries.at(n);}
};
struct Vector {
  double a=0,b=0,c=0;
  double x() const {return a;} double y() const {return b;} double z() const {return c;}
};
struct Twist {
  Header h;Vector l{.1,0,0},a;
  const auto &header() const {return h;}
  const auto &linear() const {return l;}
  const auto &angular() const {return a;}
};
#include DECODER_HEADER
int main(int argc,char **argv) {
  Twist msg;
  msg.h.entries={{"version",{"1"}},{"session_id",{"test"}},{"kind",{"command"}},
    {"command_seq",{"1"}},{"frame_id",{"f1"}},{"captured_sim_s",{"0.1"}},
    {"issued_steady_s",{"10"}},{"expires_steady_s",{"10.8"}}};
  const std::string mode=argc>1?argv[1]:"valid";
  if(mode=="count") {
    msg.h.entries.clear();
    for(int n=0;n<std::stoi(argv[2]);++n)
      msg.h.entries.push_back({"unknown"+std::to_string(n),{std::string(128,'x')}});
  } else if(mode=="key") msg.h.entries[0].k=std::string(std::stoul(argv[2]),'x');
  else if(mode=="value" || mode=="set") {
    for(auto &e:msg.h.entries) if(e.k==argv[2])
      e.values[0]=mode=="value"?std::string(std::stoul(argv[3]),'x'):argv[3];
  } else if(mode=="duplicate") msg.h.entries[std::stoi(argv[2])]=msg.h.entries[(std::stoi(argv[2])+1)%8];
  else if(mode=="unknown") msg.h.entries[std::stoi(argv[2])].k="unknown";
  else if(mode=="values") msg.h.entries[0].values.resize(std::stoul(argv[2]),"x");
  else if(mode=="axis") {
    double v=std::stod(argv[3]);int n=std::stoi(argv[2]);
    if(n==0)msg.l.b=v;
    if(n==1)msg.l.c=v;
    if(n==2)msg.a.a=v;
    if(n==3)msg.a.b=v;
  } else if(mode=="reorder") std::reverse(msg.h.entries.begin(),msg.h.entries.end());
  else if(mode=="max") {
    msg.h.entries[1].values[0]=std::string(256,'s');
    msg.h.entries[4].values[0]=std::string(256,'f');
    msg.h.entries[3].values[0]="18446744073709551615";
    msg.h.entries[5].values[0]="0."+std::string(61,'0')+"1";
  }
  measuring=true;
  auto packet=toponav::DecodeEnvelope(msg);
  measuring=false;
  auto bytes=allocated_bytes,calls=allocations,keys=key_reads,values=value_reads,touches=visits;
  toponav::ReceiverLease guard("test");
  if(packet)guard.Accept(*packet,10.);else guard.Trip("invalid_receiver_envelope");
  std::cout<<"{\"decoded\":"<<(packet?"true":"false")<<",\"allocated_bytes\":"<<bytes
    <<",\"allocations\":"<<calls<<",\"key_reads\":"<<keys<<",\"value_reads\":"<<values
    <<",\"visits\":"<<touches<<",\"latched\":"<<(guard.latched?"true":"false")
    <<",\"reason\":\""<<guard.reason<<"\",\"seq\":"<<guard.seq<<",\"v\":"<<guard.v
    <<",\"frame_size\":"<<(packet?packet->frame.size():0)<<"}\n";
}
'''


@pytest.fixture(scope='module')
def decoder(tmp_path_factory):
    folder = tmp_path_factory.mktemp('receiver-bounds')
    source = folder/'decoder.cc'
    source.write_text(HARNESS)
    binary = folder/'decoder'
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT),
                    '-DDECODER_HEADER="simulation/native/receiver_envelope.hh"', str(source),
                    '-o', str(binary)], check=True, capture_output=True)

    def run(*args):
        result = subprocess.run([str(binary), *map(str, args)], text=True, capture_output=True, check=True)
        return json.loads(result.stdout)
    return run


@pytest.mark.parametrize('count', [0, 1, 7, 9, 1024, 65536])
def test_cardinality_rejected_before_any_entry_access_or_allocation(decoder, count):
    result = decoder('count', count)
    assert not result['decoded'] and result['latched'] and result['v'] == 0
    assert result['key_reads'] == result['value_reads'] == result['visits'] == result['allocated_bytes'] == 0


@pytest.mark.parametrize('key', ['version','session_id','kind','command_seq','frame_id',
                                'captured_sim_s','issued_steady_s','expires_steady_s'])
def test_oversized_values_rejected_before_copying_any_strings(decoder, key):
    result = decoder('value', key, 1048576)
    assert not result['decoded'] and result['latched'] and result['v'] == 0
    assert result['allocated_bytes'] == 0 and result['visits'] <= 8


@pytest.mark.parametrize('key,limit', [('version',1),('session_id',256),('kind',7),('command_seq',20),
                                      ('frame_id',256),('captured_sim_s',64),('issued_steady_s',64),('expires_steady_s',64)])
def test_each_limit_is_checked_before_copy(decoder, key, limit):
    result = decoder('value', key, limit+1)
    assert not result['decoded'] and result['allocated_bytes'] == 0


@pytest.mark.parametrize('length', [33, 1048576])
def test_oversized_keys_rejected_without_comparison_or_copy(decoder, length):
    result = decoder('key', length)
    assert not result['decoded'] and result['allocated_bytes'] == 0
    assert result['key_reads'] == 1 and result['value_reads'] == 0


@pytest.mark.parametrize('mode', ['duplicate', 'unknown'])
@pytest.mark.parametrize('index', range(8))
def test_exact_keys_no_duplicates_or_unknown_fields(decoder, mode, index):
    result = decoder(mode, index)
    assert not result['decoded'] and result['latched'] and result['v'] == 0
    assert result['allocated_bytes'] == 0 and result['visits'] <= 8


@pytest.mark.parametrize('count', [0, 2, 1024])
def test_exactly_one_value_required_without_copying_values(decoder, count):
    result = decoder('values', count)
    assert not result['decoded'] and result['value_reads'] == 0 and result['allocated_bytes'] == 0


@pytest.mark.parametrize('axis', range(4))
@pytest.mark.parametrize('value', ['0.1', 'nan', 'inf'])
def test_extra_axes_fail_closed_before_metadata_traversal(decoder, axis, value):
    result = decoder('axis', axis, value)
    assert not result['decoded'] and result['visits'] == result['allocated_bytes'] == 0


@pytest.mark.parametrize('key,value', [('version','2'),('command_seq','-1'),('command_seq','1x'),
    ('command_seq','18446744073709551616'),('captured_sim_s',''),('issued_steady_s','10x'),
    ('expires_steady_s',' 10.8'),('expires_steady_s','1e999')])
def test_strict_number_and_version_parsing_before_string_copy(decoder, key, value):
    result = decoder('set', key, value)
    assert not result['decoded'] and result['allocated_bytes'] == 0


@pytest.mark.parametrize('args', [(), ('reorder',)])
def test_original_valid_command_and_field_order_preserved(decoder, args):
    result = decoder(*args)
    assert result['decoded'] and not result['latched'] and result['v'] == .1 and result['seq'] == 1
    assert result['visits'] == result['value_reads'] == 8


def test_all_maximum_width_fields_can_be_decoded(decoder):
    result = decoder('max')
    assert result['decoded'] and result['frame_size'] == 256 and result['allocated_bytes'] <= 1024
    # Transport parsing is separate from policy: wrong session/sequence still fail.
    assert result['latched'] and result['v'] == 0


def test_rejected_oversized_message_is_invalid_not_an_accepted_idle(decoder):
    result = decoder('value', 'session_id', 257)
    assert result['reason'] == 'invalid_receiver_envelope' and result['seq'] == 0


def test_decoder_header_is_part_of_build_and_run_freeze():
    from simulation.gazebo_receiver import SOURCES
    assert 'simulation/native/receiver_envelope.hh' in SOURCES
    build = (ROOT/'tools/build_gazebo_receiver.sh').read_text()
    assert "'simulation/native/receiver_envelope.hh'" in build
    runner = (ROOT/'simulation/gazebo_cross_validation.py').read_text()
    assert '*(ROOT/n for n in SOURCES)' in runner


def test_native_command_parses_before_taking_the_physics_mutex():
    source = (ROOT/'simulation/native/gazebo_receiver_interlock.cc').read_text()
    command = source.split('void Command(', 1)[1].split('void PreUpdate(', 1)[0]
    lock = command.index('std::lock_guard<std::mutex> lock(mutex_)')
    assert command.index('packet = DecodeEnvelope(msg)') < lock
    assert 'DecodeEnvelope' not in command[lock:] and 'msg.' not in command[lock:]
    assert 'guard_->Accept(*packet, Steady())' in command[lock:]
    assert 'guard_->Trip("invalid_receiver_envelope")' in command[lock:]


def test_mutated_decoder_header_refuses_cached_binary(tmp_path, monkeypatch):
    from simulation import gazebo_receiver as module
    root = tmp_path/'root'
    root.mkdir()
    sources = {}
    for name in module.SOURCES:
        dest = root/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes((ROOT/name).read_bytes())
        sources[name] = hashlib.sha256(dest.read_bytes()).hexdigest()
    binary = root/'plugin.so'
    binary.write_bytes(b'fixture')
    build = root/'build.json'
    build.write_text(json.dumps(dict(binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), sources=sources)))
    with (root/'simulation/native/receiver_envelope.hh').open('a') as stream:
        stream.write('\n// changed\n')
    monkeypatch.setattr(module, 'ROOT', root)
    monkeypatch.setattr(module, 'BINARY', binary)
    monkeypatch.setattr(module, 'BUILD', build)
    with pytest.raises(RuntimeError, match='Missing/stale'):
        module.verify_binary()
