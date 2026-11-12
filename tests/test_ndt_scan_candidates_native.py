"""Actual C++ scan scheduler with fake TF/clock and a serial mock matcher.

No ROS node, robot, or actual NDT is run. The optional CLI compares a saved
unmodified header with the current scheduler under explicit synthetic costs.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

import pytest

from ndt_realtime_test_support import HEADER, compile_cpp, method, section


def scheduler_source(source):
    if '// BEGIN_SCAN_CANDIDATES' in source:
        return section(source, 'SCAN_CANDIDATES')
    # Use the actual before-change callbacks, not a hand-written baseline.
    receive = method(source[source.index('points_sub = create_subscription'):],
                     '[this](sensor_msgs::msg::PointCloud2::SharedPtr msg)')
    timer = method(source, 'scan_timer_=create_wall_timer')
    receive = receive[receive.index('{') + 1:-1]
    timer = timer[timer.index('{') + 1:-1]
    return ('void QueueScan(const sensor_msgs::msg::PointCloud2::SharedPtr& msg){' + receive + '}'
            'void ProcessPendingScan(){' + timer + '}')


CPP = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <functional>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>
using Ns=std::int64_t;
constexpr Ns MS=1000000, EPOCH=10000000000LL;
static Ns clock_ns=EPOCH;
constexpr int RCL_STEADY_TIME=1;
namespace rclcpp {
struct Time {
 Ns ns;
 Time(int s,unsigned n,int):ns(Ns(s)*1000000000LL+n){}
 explicit Time(Ns n):ns(n){}
 double seconds()const{return ns/1e9;}
 Ns nanoseconds()const{return ns;}
};
struct Duration {Ns ns;explicit Duration(Ns n):ns(n){}};
struct Clock {explicit Clock(int){} Time now()const{return Time(clock_ns);}};
}
namespace sensor_msgs {namespace msg {
struct PointCloud2 {
 using SharedPtr=std::shared_ptr<PointCloud2>;
 struct Header {struct Stamp {int sec;unsigned nanosec;}stamp;}header;
};
}}
using Scan=sensor_msgs::msg::PointCloud2::SharedPtr;
static Ns stamp(const Scan& s){return Ns(s->header.stamp.sec)*1000000000LL+s->header.stamp.nanosec;}
static Scan scan(Ns ns){auto s=std::make_shared<sensor_msgs::msg::PointCloud2>();
 s->header.stamp.sec=ns/1000000000LL;s->header.stamp.nanosec=ns%1000000000LL;return s;}
struct Buffer {
 Ns delay=45*MS;bool available=true;int queries=0;Ns last_query=0;
 bool canTransform(const std::string& target,const std::string& source,
                   rclcpp::Time t,rclcpp::Duration timeout){
  assert(target=="robot_odom" && source=="leg_base_link" && timeout.ns==0);
  ++queries;last_query=t.ns;return available && t.ns<=clock_ns-delay;
 }
};
struct Harness {
 Scan pending_scan_,waiting_scan_;
 Ns last_received_scan_ns_=0,last_processed_scan_ns_=0;
 double last_processed_scan_=0; // Used only by the extracted legacy timer.
 std::string robot_odom_frame_id="robot_odom",robot_odom_child_frame_id="leg_base_link";
 Buffer tf_buffer;
 Ns cost=20*MS;bool imu_ok=true,lookup_ok=true;
 int attempts=0,matches=0,expired_results=0,missing_inputs=0,exact_lookups=0;
 Ns matching_work=0;
 std::vector<Ns> matched,published,ages;
 std::function<void(Ns)> arrivals;
 bool occasional_slow=false;
 void LidarCallback(const Scan& s){
  ++attempts;
  if(clock_ns-stamp(s)>500*MS || clock_ns-stamp(s)<-50*MS) return;
  ++exact_lookups;
  if(!imu_ok || !lookup_ok){++missing_inputs;return;}
  ++matches;matched.push_back(stamp(s));
  const Ns duration=occasional_slow && matches%15==0 ? 700*MS : cost;
  matching_work+=duration;
  const Ns end=clock_ns+duration;if(arrivals)arrivals(end);clock_ns=end;
  // The production validator is covered separately by the full callback test.
  if(clock_ns-stamp(s)>500*MS){++expired_results;return;}
  published.push_back(stamp(s));ages.push_back(clock_ns-stamp(s));
 }
 size_t slots()const{return size_t(bool(pending_scan_))+size_t(bool(waiting_scan_));}
 void assert_order()const{
  assert(std::set<Ns>(matched.begin(),matched.end()).size()==matched.size());
  assert(std::is_sorted(matched.begin(),matched.end()));
  assert(std::set<Ns>(published.begin(),published.end()).size()==published.size());
  assert(std::is_sorted(published.begin(),published.end()));assert(slots()<=2);
 }
 // INSERT_PRODUCTION
};
static Ns cursor(const Harness& h){return std::max(h.last_processed_scan_ns_,Ns(std::llround(h.last_processed_scan_*1e9)));}

static void scenario(const std::string& name){
 clock_ns=EPOCH;Harness h;
 if(name=="no_input"){h.ProcessPendingScan();assert(h.attempts==0 && h.tf_buffer.queries==0);}
 else if(name=="tf45"){
  h.QueueScan(scan(EPOCH));h.ProcessPendingScan();assert(h.matches==0);
  clock_ns+=50*MS;h.ProcessPendingScan();assert(h.matches==1 && h.published[0]==EPOCH);
 }
 else if(name=="tf125" || name=="tf250"){
  const int delay=name=="tf125" ? 125 : 250;h.tf_buffer.delay=delay*MS;
  h.QueueScan(scan(EPOCH));h.ProcessPendingScan();
  for(int t=100;t<=delay;t+=100){clock_ns=EPOCH+t*MS;h.QueueScan(scan(clock_ns));h.ProcessPendingScan();}
  clock_ns=EPOCH+(delay+5)*MS;h.ProcessPendingScan();
  assert(h.matches==1 && h.published[0]==EPOCH && h.ages[0]<=500*MS);
 }
 else if(name=="missing_tf_expires"){
  h.tf_buffer.available=false;h.QueueScan(scan(EPOCH));h.ProcessPendingScan();
  clock_ns+=100*MS;h.QueueScan(scan(clock_ns));h.ProcessPendingScan();
  const int before=h.tf_buffer.queries;clock_ns=EPOCH+601*MS;h.ProcessPendingScan();
  assert(h.slots()==0 && h.matches==0 && h.tf_buffer.queries==before);
 }
 else if(name=="latest_wins"){
  h.tf_buffer.available=false;h.QueueScan(scan(EPOCH));
  clock_ns+=100*MS;h.QueueScan(scan(clock_ns));
  clock_ns+=100*MS;h.tf_buffer.available=true;h.ProcessPendingScan();
  assert(h.matched.size()==1 && h.matched[0]==EPOCH+100*MS && h.slots()==0);
 }
 else if(name=="duplicates_reverse"){
  h.QueueScan(scan(EPOCH));h.ProcessPendingScan();
  clock_ns+=100*MS;h.QueueScan(scan(clock_ns));
  h.QueueScan(scan(EPOCH));h.QueueScan(scan(EPOCH-1));
  clock_ns+=100*MS;h.ProcessPendingScan();h.ProcessPendingScan();
  assert(h.matched.size()==1 && h.matched[0]==EPOCH+100*MS);
 }
 else if(name=="nanosecond_order"){
  clock_ns=100000000000000000LL;h.tf_buffer.delay=0;
  const Ns base=clock_ns;h.QueueScan(scan(base));h.ProcessPendingScan();
  h.QueueScan(scan(base+1));h.ProcessPendingScan();h.QueueScan(scan(base));h.ProcessPendingScan();
  assert(h.matched.size()==2 && h.matched[1]==base+1);
 }
 else if(name=="missing_imu" || name=="tf_disappears"){
  h.tf_buffer.delay=0;h.imu_ok=name!="missing_imu";h.lookup_ok=name!="tf_disappears";
  h.QueueScan(scan(EPOCH));h.ProcessPendingScan();h.ProcessPendingScan();
  assert(h.matches==0 && h.attempts==1 && h.published.empty());
 }
 else if(name=="slow700"){
  h.cost=700*MS;h.tf_buffer.available=false;h.QueueScan(scan(EPOCH));
  clock_ns+=100*MS;h.QueueScan(scan(clock_ns));clock_ns+=50*MS;
  h.tf_buffer.available=true;h.tf_buffer.delay=100*MS;
  h.ProcessPendingScan();assert(h.matches==1 && h.published.empty() && h.slots()==0);
 }
 else if(name=="arrivals_during_match"){
  h.tf_buffer.delay=0;h.cost=250*MS;Scan reader;
  h.arrivals=[&](Ns end){for(Ns t=EPOCH+100*MS;t<=end;t+=100*MS)reader=scan(t);};
  h.QueueScan(scan(EPOCH));h.ProcessPendingScan();
  assert(h.matches==1 && reader && stamp(reader)==EPOCH+200*MS);
  h.arrivals={};h.cost=20*MS;h.QueueScan(reader);h.ProcessPendingScan();
  assert(h.matches==2 && h.matched[1]==EPOCH+200*MS);
 }
 else if(name=="callback_orders"){
  for(bool receive_first:{false,true}){
   clock_ns=EPOCH;Harness x;x.tf_buffer.delay=0;x.QueueScan(scan(EPOCH));
   clock_ns+=100*MS;
   if(receive_first)x.QueueScan(scan(clock_ns));x.ProcessPendingScan();
   if(!receive_first)x.QueueScan(scan(EPOCH+100*MS));x.ProcessPendingScan();
   x.assert_order();assert(x.matches<=2);
  }
 }
 else if(name=="one_match_per_tick"){
  h.tf_buffer.available=false;h.QueueScan(scan(EPOCH));clock_ns+=100*MS;h.QueueScan(scan(clock_ns));
  clock_ns+=50*MS;h.tf_buffer.available=true;h.tf_buffer.delay=100*MS;
  int queries=h.tf_buffer.queries;h.ProcessPendingScan();
  assert(h.matches==1 && h.tf_buffer.queries-queries<=2);
  h.ProcessPendingScan();assert(h.matches==1);
 }
 else if(name=="bounded"){
  h.tf_buffer.available=false;
  for(int i=0;i<1000;++i){clock_ns=EPOCH+i*10*MS;h.QueueScan(scan(clock_ns));h.ProcessPendingScan();assert(h.slots()<=2);}
  assert(h.matches==0);
 }
 else if(name=="received_expired"){
  clock_ns+=501*MS;h.QueueScan(scan(EPOCH));h.ProcessPendingScan();assert(h.slots()==0 && h.matches==0 && h.tf_buffer.queries==0);
 }
 else if(name=="invalid_stamp"){
  auto malformed=scan(EPOCH);malformed->header.stamp.nanosec=1000000000U;
  h.QueueScan(malformed);h.QueueScan(scan(EPOCH+51*MS));h.ProcessPendingScan();
  assert(h.slots()==0 && h.matches==0 && h.tf_buffer.queries==0);
 }
 else if(name=="remaining_budget"){
  h.tf_buffer.delay=100*MS;h.cost=250*MS;
  h.QueueScan(scan(EPOCH));clock_ns+=100*MS;h.QueueScan(scan(clock_ns));
  clock_ns+=50*MS;h.ProcessPendingScan();
  assert(h.matches==1 && h.published[0]==EPOCH && h.slots()==0);
  // The stale unselected latest was coalesced, not made into the next waiter.
  // A future input is still admitted after a slow or rejected correction.
  h.cost=20*MS;const Ns next=clock_ns;h.QueueScan(scan(next));
  clock_ns+=100*MS;h.ProcessPendingScan();
  assert(h.matches==2 && h.published.back()==next && h.expired_results==0);
 }
 else assert(false);
 h.assert_order();
}

static void benchmark(int delay_ms,int cost_ms,bool occasional_slow){
 clock_ns=EPOCH;Harness h;h.tf_buffer.delay=delay_ms*MS;h.cost=cost_ms*MS;h.occasional_slow=occasional_slow;
 Ns next_source=EPOCH;const Ns end=EPOCH+30000*MS;Scan reader;
 int received=0,max_slots=0,overwrites=0,loops=0,max_queries=0,max_matches=0;
 int max_total_queries=0;
 std::set<Ns> expired_candidates,budget_coalesced;
 auto slots=[&](){return std::vector<Scan>{h.pending_scan_,h.waiting_scan_};};
 auto removed=[&](const std::vector<Scan>& previous,bool after_processing){
  for(const auto& s:previous){
   if(!s || s==h.pending_scan_ || s==h.waiting_scan_ ||
      std::find(h.matched.begin(),h.matched.end(),stamp(s))!=h.matched.end())continue;
   if(clock_ns-stamp(s)>500*MS)expired_candidates.insert(stamp(s));
   else if(after_processing && stamp(s)>cursor(h))budget_coalesced.insert(stamp(s));
  }
 };
 auto arrivals=[&](Ns until){while(next_source<=until && next_source<end){reader=scan(next_source);next_source+=100*MS;++received;}};
 h.arrivals=arrivals;
 const auto cpu_start=std::clock();
 while(clock_ns<end){
  arrivals(clock_ns);
  Scan protected_before=h.waiting_scan_ ? h.waiting_scan_ : h.pending_scan_;
  const auto before_receive=slots();
  if(reader){h.QueueScan(reader);reader.reset();
   if(protected_before && stamp(protected_before)>cursor(h) &&
      clock_ns-stamp(protected_before)<=500*MS && h.pending_scan_!=protected_before && h.waiting_scan_!=protected_before)++overwrites;
  }
  removed(before_receive,false);
  max_slots=std::max(max_slots,int(h.slots()));
  const auto before_timer=slots();
  const int q=h.tf_buffer.queries,m=h.matches,l=h.exact_lookups;const Ns tick=clock_ns;
  h.ProcessPendingScan();h.assert_order();
  removed(before_timer,true);
  max_queries=std::max(max_queries,h.tf_buffer.queries-q);max_matches=std::max(max_matches,h.matches-m);
  max_total_queries=std::max(max_total_queries,h.tf_buffer.queries-q+h.exact_lookups-l);
  clock_ns=std::max(clock_ns,tick+10*MS);++loops;
 }
 double cpu_ms=1000.*(std::clock()-cpu_start)/CLOCKS_PER_SEC;
 std::cout<<"{\"received\":"<<received<<",\"ndt_calls\":"<<h.matches<<",\"published\":"<<h.published.size()
  <<",\"expired_results\":"<<h.expired_results<<",\"max_pending_slots\":"<<max_slots
  <<",\"expired_candidate_drops\":"<<expired_candidates.size()<<",\"budget_coalesced\":"<<budget_coalesced.size()
  <<",\"waiting_candidate_overwrites\":"<<overwrites
  <<",\"tf_readiness_probes\":"<<h.tf_buffer.queries<<",\"tf_exact_lookups\":"<<h.exact_lookups
  <<",\"tf_queries\":"<<h.tf_buffer.queries+h.exact_lookups
  <<",\"max_tf_readiness_probes_per_tick\":"<<max_queries<<",\"max_tf_queries_per_tick\":"<<max_total_queries
  <<",\"max_ndt_calls_per_tick\":"<<max_matches<<",\"simulated_matching_work_ms\":"<<h.matching_work/MS
  <<",\"native_mock_cpu_ms\":"<<cpu_ms<<",\"timer_calls\":"<<loops<<"}\n";
}
int main(int argc,char** argv){
 if(argc>1 && std::string(argv[1])=="bench"){benchmark(std::stoi(argv[2]),std::stoi(argv[3]),std::stoi(argv[4]));return 0;}
 scenario(argv[1]);std::cout<<"OK\n";
}
'''


def build_scheduler(directory, source, name='scan_scheduler'):
    code = CPP.replace('// INSERT_PRODUCTION', section(source, 'SCAN_READINESS') + scheduler_source(source))
    return compile_cpp(directory, name, code)


@pytest.fixture(scope='module')
def scheduler(tmp_path_factory):
    return build_scheduler(tmp_path_factory.mktemp('ndt_scheduler'), HEADER.read_text())


@pytest.mark.parametrize('scenario', [
    'no_input', 'tf45', 'tf125', 'tf250', 'missing_tf_expires', 'latest_wins',
    'duplicates_reverse', 'nanosecond_order', 'missing_imu', 'tf_disappears',
    'slow700', 'arrivals_during_match', 'callback_orders', 'one_match_per_tick',
    'bounded', 'received_expired', 'invalid_stamp', 'remaining_budget',
])
def test_production_scan_scheduler(scheduler, scenario):
    result = subprocess.run([str(scheduler), scenario], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def compare(baseline, directory):
    before = build_scheduler(directory, Path(baseline).read_text(), 'scan_before')
    after = build_scheduler(directory, HEADER.read_text(), 'scan_after')
    result = {}
    for name, delay, cost, occasional in [
        ('fast45', 45, 20, 0), ('fast125', 125, 20, 0), ('fast250', 250, 50, 0),
        ('slow250_tf45', 45, 250, 0), ('slow250_tf125', 125, 250, 0), ('slow250_tf250', 250, 250, 0),
        ('occasional700_tf45', 45, 250, 1), ('occasional700_tf125', 125, 250, 1),
        ('sustained700_tf45', 45, 700, 0), ('sustained700_tf125', 125, 700, 0),
        ('missing_tf', 10000, 250, 0),
    ]:
        row = {}
        for role, binary in [('before', before), ('after', after)]:
            p = subprocess.run([str(binary), 'bench', str(delay), str(cost), str(occasional)],
                               capture_output=True, text=True, check=True)
            row[role] = json.loads(p.stdout)
        result[name] = row
    return dict(scope='30-second deterministic native mock; actual scheduler code, synthetic TF and NDT cost; no ROS/DDS/PCL registration performance claim',
                baseline=str(baseline), scenarios=result)


def test_saved_baseline_comparison(tmp_path):
    baseline = os.environ.get('LITE3_NDT_BASELINE_HEADER')
    if not baseline:
        pytest.skip('set LITE3_NDT_BASELINE_HEADER to an unmodified header for before/after comparison')
    result = compare(baseline, tmp_path)
    assert result['scenarios']['fast125']['before']['published'] == 0
    assert result['scenarios']['fast125']['after']['published'] > 0
    slow = result['scenarios']['slow250_tf125']['after']
    assert slow['published'] > 0 and slow['expired_results'] == 0
    assert slow['budget_coalesced'] > 0
    assert result['scenarios']['slow250_tf45']['before']['ndt_calls'] == result['scenarios']['slow250_tf45']['after']['ndt_calls']
    for row in result['scenarios'].values():
        assert row['after']['max_pending_slots'] <= 2
        assert row['after']['max_tf_readiness_probes_per_tick'] <= 2
        assert row['after']['max_tf_queries_per_tick'] <= 3  # Plus existing exact lookup inside LidarCallback.
        assert row['after']['max_ndt_calls_per_tick'] <= 1
    assert result['scenarios']['sustained700_tf125']['after']['published'] == 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as directory:
        report = compare(args.baseline, directory)
    with args.output.open('x') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(args.output)
