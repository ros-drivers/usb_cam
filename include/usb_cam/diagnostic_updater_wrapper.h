// Copyright 2019-2022 Miso Robotics, Inc.

#ifndef MISOCPP_DIAGNOSTIC_UPDATER_WRAPPER_H
#define MISOCPP_DIAGNOSTIC_UPDATER_WRAPPER_H

#include <memory>
#include <string>

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <ros/ros.h>

namespace misocpp
{
//! \brief Wrapper class for heartbeat diagnostics.
class DiagnosticHeartbeat
{
public:
  //! \brief Constructor for DiagnosticHeartbeat.
  //! \param hwid String to set as hardware ID.
  explicit DiagnosticHeartbeat(const std::string& hwid = DEFAULT_HARDWARE_ID)
  {
    diagnostic_updater_.setHardwareID(hwid);
    diagnostic_updater_.add(heartbeat_);
  }

  //! \brief Call update() function of Updater class object.
  void update()
  {
    diagnostic_updater_.update();
  }

private:
  inline static const std::string DEFAULT_HARDWARE_ID{ "none" };
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::Heartbeat heartbeat_;
};

//! \brief Wrapper class for frequency diagnostics.
class DiagnosticFrequency : public diagnostic_updater::DiagnosticTask
{
public:
  //! \brief Constructor for DiagnosticFrequency.
  //! \param topic_name The name of the topic that is being diagnosed.
  //! \param min_freq Minimum acceptable frequency.
  //! \param max_freq Minimum acceptable frequency.
  //! \param tolerance Tolerance with which bounds must be satisfied.
  //! \param window_size Number of events to consider in the statistics.
  //! \param hwid String to set as hardware ID.
  //! \param suffix The string to add to the monitoring name.
  explicit DiagnosticFrequency(const std::string& topic_name, const double& min_freq, const double& max_freq,
                               const double tolerance = 0.1, const int window_size = 5,
                               const std::string& hwid = DEFAULT_HARDWARE_ID,
                               const std::string& suffix = ": FrequencyStatus")
    : DiagnosticTask(topic_name + suffix)
  {
    ROS_ASSERT(min_freq <= max_freq);
    min_freq_ = min_freq;
    max_freq_ = max_freq;

    frequency_status_param_ =
        std::make_unique<diagnostic_updater::FrequencyStatusParam>(&min_freq_, &max_freq_, tolerance, window_size);
    ROS_ASSERT(frequency_status_param_);

    frequency_status_ = std::make_unique<diagnostic_updater::FrequencyStatus>(*frequency_status_param_);
    ROS_ASSERT(frequency_status_);

    task_ = (diagnostic_updater::DiagnosticTask*)frequency_status_.get();
    diagnostic_updater_.setHardwareID(hwid);
    diagnostic_updater_.add(*this);
  }

  //! \brief Fills out this Task's DiagnosticStatusWrapper.
  //! \param stat The DiagnosticStatusWrapper to fill.
  virtual void run(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    task_->run(stat);
  }

  //! \brief Signal that a publication has occurred.
  //! \param call_update Flag whether Updater.update() method is called.
  virtual void tick(bool call_update = true)
  {
    frequency_status_->tick();
    if (call_update)
      diagnostic_updater_.update();
  }

  //! \brief Clear the frequency statistics.
  virtual void clear_window()
  {
    frequency_status_->clear();
  }

private:
  double min_freq_{ 0.0 }, max_freq_{ 0.0 };
  std::unique_ptr<diagnostic_updater::FrequencyStatusParam> frequency_status_param_{ nullptr };
  std::unique_ptr<diagnostic_updater::FrequencyStatus> frequency_status_{ nullptr };
  inline static const std::string DEFAULT_HARDWARE_ID{ "none" };
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::DiagnosticTask* task_{ nullptr };
};
}  // namespace misocpp
#endif  // MISOCPP_DIAGNOSTIC_UPDATER_WRAPPER_H
