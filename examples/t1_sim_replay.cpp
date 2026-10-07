/* ----------------------------------------------------------------------------
 * T1-specific addition (not part of upstream UMich-CURLY/drift).
 * -------------------------------------------------------------------------- */

/**
 *  @file   t1_sim_replay.cpp
 *  @brief  Replays a CSV produced by booster-mjlab's
 *          scripts/replay_t1_for_drift.py (IMU, joint encoders, and the
 *          LEARNED contact estimator's per-step output) through a real
 *          InekfEstimator configured with T1Kinematics, then writes the
 *          estimated trajectory next to the simulator's own ground-truth
 *          base pose for comparison.
 *
 *          No ROS: measurements are pushed one row at a time into the same
 *          queues the ROS subscriber would otherwise fill, immediately
 *          followed by one RunOnce() -- DRIFT's correction classes are not
 *          designed to have a whole trajectory pre-loaded into their queues
 *          (their one-shot `initialize()` discards all but the latest queued
 *          message), so this mimics a live producer one sample at a time
 *          instead.
 *
 *  Usage:
 *    T1_MJCF_PATH=<path to t1.xml> ./t1_sim_replay <input.csv> <output.csv>
 **/

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "drift/estimator/inekf_estimator.h"
#include "drift/kinematics/t1_kinematics.h"

using namespace measurement;
using namespace estimator;
using namespace t1_kinematics;

namespace {

struct CsvTable {
  std::vector<std::string> header;
  std::vector<std::vector<double>> rows;
  std::map<std::string, int> col;

  double at(size_t row, const std::string& name) const {
    return rows[row][col.at(name)];
  }
};

// Strips a trailing '\r' left behind when a CRLF-terminated file (e.g.
// Python's csv module, which writes \r\n by default per the CSV spec) is
// read with std::getline, which only splits on '\n'.
std::string StripCR(std::string s) {
  if (!s.empty() && s.back() == '\r') s.pop_back();
  return s;
}

CsvTable ReadCsv(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("Could not open CSV: " + path);
  }
  CsvTable table;
  std::string line;
  std::getline(f, line);
  line = StripCR(line);
  std::stringstream header_ss(line);
  std::string cell;
  int idx = 0;
  while (std::getline(header_ss, cell, ',')) {
    table.header.push_back(cell);
    table.col[cell] = idx++;
  }
  while (std::getline(f, line)) {
    line = StripCR(line);
    if (line.empty()) continue;
    std::stringstream row_ss(line);
    std::vector<double> row;
    row.reserve(table.header.size());
    while (std::getline(row_ss, cell, ',')) {
      row.push_back(std::stod(cell));
    }
    table.rows.push_back(std::move(row));
  }
  return table;
}

Eigen::Matrix<double, 12, 1> ReadLegVector(const CsvTable& t, size_t row,
                                            const std::string& prefix) {
  Eigen::Matrix<double, 12, 1> v;
  for (int i = 0; i < 12; ++i) {
    v[i] = t.at(row, prefix + "_" + std::to_string(i));
  }
  return v;
}

}    // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <input.csv> <output.csv>"
              << std::endl;
    return 1;
  }
  const std::string in_path = argv[1];
  const std::string out_path = argv[2];

  CsvTable table = ReadCsv(in_path);
  const size_t n = table.rows.size();
  std::cout << "[INFO]: Loaded " << n << " rows from " << in_path
            << std::endl;
  if (n < 2) {
    std::cerr << "Need at least 2 rows (1 for init, 1+ to replay)."
              << std::endl;
    return 1;
  }

  inekf::ErrorType error_type = LeftInvariant;
  InekfEstimator inekf_estimator(error_type, "../config/t1/inekf_estimator.yaml");

  IMUQueue imu_buffer;
  IMUQueuePtr imu_buffer_ptr = std::make_shared<IMUQueue>(imu_buffer);
  LeggedKinQueue kin_buffer;
  LeggedKinQueuePtr kin_buffer_ptr
      = std::make_shared<LeggedKinQueue>(kin_buffer);
  auto imu_mutex = std::make_shared<std::mutex>();
  auto kin_mutex = std::make_shared<std::mutex>();

  inekf_estimator.add_imu_propagation(imu_buffer_ptr, imu_mutex,
                                      "../config/t1/imu_propagation.yaml");
  inekf_estimator.add_legged_kinematics_correction(
      kin_buffer_ptr, kin_mutex,
      "../config/t1/legged_kinematics_correction.yaml");

  auto push_row = [&](size_t i) {
    ImuMeasurement<double> imu;
    imu.set_angular_velocity(table.at(i, "imu_gyro_x"),
                             table.at(i, "imu_gyro_y"),
                             table.at(i, "imu_gyro_z"));
    imu.set_lin_acc(table.at(i, "imu_acc_x"), table.at(i, "imu_acc_y"),
                    table.at(i, "imu_acc_z"));
    imu.set_time(table.at(i, "t"));
    imu_buffer_ptr->push(std::make_shared<ImuMeasurement<double>>(imu));

    Eigen::Matrix<double, 12, 1> encoders = ReadLegVector(table, i, "q");
    Eigen::Matrix<double, 12, 1> d_encoders = ReadLegVector(table, i, "dq");
    Eigen::Matrix<bool, 2, 1> contacts;
    // Fed by the LEARNED contact estimator's output, not ground truth -- this
    // is the whole point of the replay.
    contacts << (table.at(i, "contact_est_left") > 0.5),
        (table.at(i, "contact_est_right") > 0.5);
    auto kin = std::make_shared<kinematics::T1Kinematics>(encoders, d_encoders,
                                                           contacts);
    kin->set_time(table.at(i, "t"));
    kin_buffer_ptr->push(kin);
  };

  // Seed the queues with exactly one sample before Init: LeggedKinematics-
  // Correction::initialize() discards all but the LAST queued message, so
  // pre-loading the whole trajectory here would silently throw away all but
  // one row.
  push_row(0);
  while (!inekf_estimator.is_enabled()) {
    if (inekf_estimator.BiasInitialized()) {
      inekf_estimator.InitState();
      inekf_estimator.EnableFilter();
    } else {
      inekf_estimator.InitBias();
    }
  }

  // Ground-truth yaw from base_quat_{w,x,y,z} (same convention as the
  // estimated yaw below: atan2 of the standard ZYX heading term).
  auto gt_yaw = [&](size_t i) {
    double w = table.at(i, "base_quat_w"), x = table.at(i, "base_quat_x"),
           y = table.at(i, "base_quat_y"), z = table.at(i, "base_quat_z");
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  };

  std::ofstream out(out_path);
  out << "t,est_pos_x,est_pos_y,est_pos_z,gt_pos_x,gt_pos_y,gt_pos_z,"
      << "est_vel_x,est_vel_y,est_vel_z,gt_vel_x,gt_vel_y,gt_vel_z,"
      << "est_yaw,gt_yaw\n";
  auto log_row = [&](size_t i) {
    const RobotState state = inekf_estimator.get_state();
    const Eigen::Vector3d p = state.get_position();
    const Eigen::Vector3d v = state.get_velocity();
    const Eigen::Matrix3d R = state.get_rotation();
    const double est_yaw = std::atan2(R(1, 0), R(0, 0));
    out << table.at(i, "t") << "," << p.x() << "," << p.y() << "," << p.z()
        << "," << table.at(i, "base_pos_x") << "," << table.at(i, "base_pos_y")
        << "," << table.at(i, "base_pos_z") << "," << v.x() << "," << v.y()
        << "," << v.z() << "," << table.at(i, "base_vel_x") << ","
        << table.at(i, "base_vel_y") << "," << table.at(i, "base_vel_z") << ","
        << est_yaw << "," << gt_yaw(i) << "\n";
  };
  log_row(0);

  for (size_t i = 1; i < n; ++i) {
    push_row(i);
    inekf_estimator.RunOnce();
    log_row(i);
    if (i % 500 == 0) {
      std::cout << "  step " << i << "/" << n << std::endl;
    }
  }
  out.close();
  std::cout << "[INFO]: Wrote estimated vs. ground-truth trajectory to "
            << out_path << std::endl;
  return 0;
}
