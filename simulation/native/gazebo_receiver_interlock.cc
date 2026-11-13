// Final wheel command interlock, AFTER original DiffDrive PreUpdate (priority 0).
// Physics runs in Update, after every sequential PreUpdate. No competing Twist
// publisher and no dependence on navigation, audit or velocity worker liveness.
#include "receiver_envelope.hh"
#include "receiver_status.hh"
#include <ctime>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>

namespace toponav {
static double Steady() {
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return std::numeric_limits<double>::quiet_NaN();
  return static_cast<double>(ts.tv_sec)+ts.tv_nsec/1e9;
}
class ReceiverInterlock final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemConfigurePriority,
    public gz::sim::ISystemPreUpdate {
 public:
  PriorityType ConfigurePriority() override { return 1; }
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    session_ = sdf->Get<std::string>("session_id");
    if (session_.empty() || session_.size() > 256 || session_.find_first_not_of(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_") != std::string::npos)
      throw std::invalid_argument("Invalid receiver session");
    gz::sim::Model model(entity);
    left_ = model.JointByName(ecm, "left_wheel_joint");
    right_ = model.JointByName(ecm, "right_wheel_joint");
    if (!left_ || !right_) throw std::invalid_argument("Missing receiver wheel joints");
    guard_ = std::make_unique<ReceiverLease>(session_);
    Write(ecm, left_, 0); Write(ecm, right_, 0);
    status_ = node_.Advertise<gz::msgs::StringMsg>("/toponav/receiver_status");
    if (!status_ || !node_.Subscribe("/toponav/cmd_vel", &ReceiverInterlock::Command, this))
      throw std::runtime_error("Receiver transport unavailable");
  }
  void Command(const gz::msgs::Twist &msg) {
    // No message traversal, string copying or parsing under the physics/lease
    // mutex. Native publishers do not pass through the Python packet bounds.
    std::optional<Envelope> packet;
    try {
      packet = DecodeEnvelope(msg);
    } catch (const std::exception &) { /* Allocation failure also fails closed. */ }
    std::lock_guard<std::mutex> lock(mutex_);
    if (packet) guard_->Accept(*packet, Steady());
    else guard_->Trip("invalid_receiver_envelope");
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now=Steady();
    guard_->Poll(now);
    // Original fixture: wheel separation .30m, radius .08m. No pose feedback.
    // Always write the FINAL authorized pair, not merely race a stop publisher
    // against DiffDrive or allow an unvalidated Twist callback to win a step.
    Write(ecm, left_, (guard_->v-guard_->w*.15)/.08);
    Write(ecm, right_, (guard_->v+guard_->w*.15)/.08);
    const double sim=std::chrono::duration<double>(info.simTime).count();
    // Paused worlds still poll the wall deadline, without flooding status
    // while neither physics nor authority changed.
    if (sim == last_sim_ && guard_->seq == last_seq_ && guard_->reason == last_reason_) return;
    last_sim_=sim; last_seq_=guard_->seq; last_reason_=guard_->reason;
    gz::msgs::StringMsg status;
    status.set_data(ReceiverStatus(*guard_, session_, now, sim));
    status_.Publish(status);  // Diagnostic only, NOT physical-stop confirmation.
  }
 private:
  static void Write(gz::sim::EntityComponentManager &ecm, gz::sim::Entity joint, double value) {
    auto *component=ecm.Component<gz::sim::components::JointVelocityCmd>(joint);
    if (!component) ecm.CreateComponent(joint, gz::sim::components::JointVelocityCmd({value}));
    else component->Data()={value};
  }
  std::string session_;
  gz::sim::Entity left_=gz::sim::kNullEntity, right_=gz::sim::kNullEntity;
  std::unique_ptr<ReceiverLease> guard_;
  std::mutex mutex_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher status_;
  double last_sim_=-1;
  std::uint64_t last_seq_=0;
  std::string last_reason_;
};
}  // namespace toponav
GZ_ADD_PLUGIN(toponav::ReceiverInterlock, gz::sim::System,
    toponav::ReceiverInterlock::ISystemConfigure,
    toponav::ReceiverInterlock::ISystemConfigurePriority,
    toponav::ReceiverInterlock::ISystemPreUpdate)
