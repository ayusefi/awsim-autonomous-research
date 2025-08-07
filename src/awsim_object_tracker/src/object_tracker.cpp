#include "awsim_object_tracker/object_tracker.hpp"
#include "awsim_object_tracker/perception_pipeline.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <cmath>

namespace awsim_object_tracker {

Track::Track(int id, 
              std::shared_ptr<KalmanFilter> kf, 
              TrackState state,
              int hits,
              int time_since_update,
              int age,
              std::shared_ptr<Detection> detection)
  : id(id),
    kf(kf),
    state(state),
    hits(hits),
    time_since_update(time_since_update),
    age(age),
    last_detection(detection),
    creation_time(std::chrono::steady_clock::now()),
    orientation(detection ? detection->orientation : Eigen::Quaterniond::Identity()),
    angular_velocity(Eigen::Vector3d::Zero()) {
  
  // Initialize orientation from detection if available
  if (detection) {
    orientation = detection->orientation;
  }
}Eigen::Vector3d Track::getPosition() const {
  return kf->getPosition();
}

Eigen::Vector3d Track::getVelocity() const {
  return kf->getVelocity();
}

Eigen::Quaterniond Track::getOrientation() const {
  return orientation;
}

Eigen::Vector3d Track::getAngularVelocity() const {
  return angular_velocity;
}

void Track::updateOrientation(const Eigen::Quaterniond& new_orientation, double smoothing_factor) {
  // Ensure the quaternion has the same hemisphere (prevent 180-degree flips)
  Eigen::Quaterniond aligned_orientation = new_orientation;
  if (orientation.dot(new_orientation) < 0) {
    aligned_orientation = Eigen::Quaterniond(-new_orientation.w(), -new_orientation.x(), 
                                           -new_orientation.y(), -new_orientation.z());
  }
  
  // Use SLERP for smooth interpolation
  orientation = orientation.slerp(smoothing_factor, aligned_orientation);
  orientation.normalize();
}

void Track::predict() {
  kf->predict();
  time_since_update++;
  age++;
}

void Track::update(const std::shared_ptr<Detection>& detection) {
  kf->update(detection->center);
  hits++;
  time_since_update = 0;
  last_detection = detection;
  
  // Update orientation with improved smoothing using velocity direction
  if (hits == 1) {
    // First update, set orientation based on velocity if available
    {
      Eigen::Vector3d vel = kf->getVelocity();
      if (vel.norm() > 1e-3) {
        double yaw = std::atan2(vel.y(), vel.x());
        orientation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
      }
    }
    angular_velocity = Eigen::Vector3d::Zero();
  } else {
    // Store previous orientation for angular velocity calculation
    Eigen::Quaterniond prev_orientation = orientation;
    
    // Use adaptive smoothing factor based on track confidence
    double base_smoothing = 0.15;  // More conservative smoothing
    double confidence_factor = std::min(1.0, hits / 10.0);  // Gain confidence over time
    double adaptive_smoothing = base_smoothing + (0.2 * confidence_factor);
    
    // Determine new orientation based on velocity direction
    Eigen::Vector3d vel = kf->getVelocity();
    // Prevent orientation jitter for near-zero velocities
    const double vel_thresh = 0.1;  // m/s threshold for stationary
    Eigen::Quaterniond new_orientation;
    if (vel.norm() > vel_thresh) {
      double yaw = std::atan2(vel.y(), vel.x());
      new_orientation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    } else {
      // Keep previous orientation when stationary
      new_orientation = prev_orientation;
      angular_velocity = Eigen::Vector3d::Zero();
    }
    // Apply orientation smoothing
    updateOrientation(new_orientation, adaptive_smoothing);
    
    // Calculate angular velocity (simplified approximation)
    Eigen::Quaterniond q_diff = orientation * prev_orientation.inverse();
    q_diff.normalize();
    
    // Convert to angle-axis and compute angular velocity
    Eigen::AngleAxisd angle_axis(q_diff);
    double angle = angle_axis.angle();
    
    // Normalize angle to [-π, π]
    if (angle > M_PI) {
      angle -= 2 * M_PI;
    }
    
    double dt = 0.1;  // Assume 10Hz update rate
    angular_velocity = angle_axis.axis() * angle / dt;
    
    // Limit angular velocity to reasonable values (prevent noise)
    double max_angular_vel = 2.0;  // rad/s
    if (angular_velocity.norm() > max_angular_vel) {
      angular_velocity = angular_velocity.normalized() * max_angular_vel;
    }
  }
  
  // Update track state
  if (state == TrackState::TENTATIVE && hits >= 3) {
    state = TrackState::CONFIRMED;
  }
}

void Track::markMissed() {
  time_since_update++;
  age++;
  
  // Mark for deletion if missed for too long
  if (time_since_update > 5) {
    state = TrackState::DELETED;
  }
}

MultiObjectTracker::MultiObjectTracker(double max_distance, double dt)
  : max_distance_(max_distance),
    dt_(dt),
    track_id_counter_(0),
    frame_count_(0) {}

void MultiObjectTracker::update(const std::vector<std::shared_ptr<Detection>>& detections) {
  // Step 1: Predict all tracks
  predict();
  
  // Step 2: Data association using Hungarian algorithm
  auto [matches, unmatched_detection_indices, unmatched_track_indices] = 
    associateDetectionsToTracks(detections);
  
  // Step 3: Update matched tracks
  updateTracks(detections, matches);
  
  // Step 4: Mark missed tracks
  markMissedTracks(unmatched_track_indices);
  
  // Step 5: Create new tracks
  initiateNewTracks(detections, unmatched_detection_indices);
  
  // Step 6: Delete old tracks
  deleteOldTracks();
  
  frame_count_++;
}

std::vector<std::shared_ptr<Track>> MultiObjectTracker::getActiveTracks() const {
  std::vector<std::shared_ptr<Track>> active_tracks;
  for (const auto& track : tracks_) {
    if (track->state == TrackState::CONFIRMED) {
      active_tracks.push_back(track);
    }
  }
  return active_tracks;
}

std::vector<std::shared_ptr<Track>> MultiObjectTracker::getAllTracks() const {
  return tracks_;
}

std::map<int, Eigen::Vector3d> MultiObjectTracker::getTrackPositions() const {
  std::map<int, Eigen::Vector3d> positions;
  for (const auto& track : tracks_) {
    positions[track->id] = track->getPosition();
  }
  return positions;
}

std::vector<Eigen::Vector3d> MultiObjectTracker::getTrackHistory(int track_id) const {
  auto it = track_history_.find(track_id);
  if (it != track_history_.end()) {
    return it->second;
  }
  return {};
}

void MultiObjectTracker::predict() {
  for (auto& track : tracks_) {
    track->predict();
  }
}

Eigen::MatrixXd MultiObjectTracker::calculateCostMatrix(
    const std::vector<std::shared_ptr<Detection>>& detections) const {
  if (tracks_.empty() || detections.empty()) {
    return Eigen::MatrixXd();
  }
  
  Eigen::MatrixXd cost_matrix(tracks_.size(), detections.size());
  
  for (size_t i = 0; i < tracks_.size(); ++i) {
    Eigen::Vector3d track_pos = tracks_[i]->getPosition();
    for (size_t j = 0; j < detections.size(); ++j) {
      // Use Euclidean distance as cost
      cost_matrix(i, j) = (track_pos - detections[j]->center).norm();
    }
  }
  
  return cost_matrix;
}

std::tuple<std::vector<std::pair<int, int>>, std::vector<int>, std::vector<int>> 
MultiObjectTracker::associateDetectionsToTracks(
    const std::vector<std::shared_ptr<Detection>>& detections) {
  std::vector<std::pair<int, int>> matches;
  std::vector<int> unmatched_detection_indices;
  std::vector<int> unmatched_track_indices;
  
  // Initialize unmatched detection indices (all detections start unmatched)
  for (size_t i = 0; i < detections.size(); ++i) {
    unmatched_detection_indices.push_back(i);
  }
  
  // If no tracks exist, all detections are unmatched
  if (tracks_.empty()) {
    return {matches, unmatched_detection_indices, unmatched_track_indices};
  }
  
  // If no detections, all tracks are unmatched
  if (detections.empty()) {
    for (size_t i = 0; i < tracks_.size(); ++i) {
      unmatched_track_indices.push_back(i);
    }
    return {matches, {}, unmatched_track_indices};
  }
  
  // Calculate cost matrix
  Eigen::MatrixXd cost_matrix = calculateCostMatrix(detections);
  
  // Solve assignment problem using Hungarian algorithm
  // We'll use a simple greedy approach for now (for simplicity)
  // In a real implementation, you would use the Hungarian algorithm
  
  // Initialize unmatched track indices
  for (size_t i = 0; i < tracks_.size(); ++i) {
    unmatched_track_indices.push_back(i);
  }
  
  // Greedy assignment (for simplicity)
  while (!unmatched_detection_indices.empty() && !unmatched_track_indices.empty()) {
    double min_cost = std::numeric_limits<double>::max();
    std::pair<int, int> best_match = {-1, -1};
    
    // Find the best match
    for (int track_idx : unmatched_track_indices) {
      for (int det_idx : unmatched_detection_indices) {
        if (cost_matrix(track_idx, det_idx) < min_cost) {
          min_cost = cost_matrix(track_idx, det_idx);
          best_match = {track_idx, det_idx};
        }
      }
    }
    
    // Check if the best match is within the distance threshold
    if (best_match.first != -1 && best_match.second != -1 && 
        min_cost < max_distance_) {
      matches.push_back(best_match);
      
      // Remove from unmatched lists
      unmatched_detection_indices.erase(
        std::remove(unmatched_detection_indices.begin(), 
                   unmatched_detection_indices.end(), 
                   best_match.second),
        unmatched_detection_indices.end());
      
      unmatched_track_indices.erase(
        std::remove(unmatched_track_indices.begin(), 
                   unmatched_track_indices.end(), 
                   best_match.first),
        unmatched_track_indices.end());
    } else {
      // No more valid matches
      break;
    }
  }
  
  return {matches, unmatched_detection_indices, unmatched_track_indices};
}

void MultiObjectTracker::initiateNewTracks(
    const std::vector<std::shared_ptr<Detection>>& detections, 
    const std::vector<int>& unmatched_detection_indices) {
  for (int det_idx : unmatched_detection_indices) {
    auto detection = detections[det_idx];
    
    // Create new Kalman filter
    auto kf = std::make_shared<KalmanFilter>(dt_);
    kf->initialize(detection->center);
    
    // Create new track
    auto track = std::make_shared<Track>(
      track_id_counter_,
      kf,
      TrackState::TENTATIVE,
      1,
      0,
      1,
      detection
    );
    
    tracks_.push_back(track);
    track_history_[track_id_counter_] = {detection->center};
    track_id_counter_++;
  }
}

void MultiObjectTracker::updateTracks(
    const std::vector<std::shared_ptr<Detection>>& detections, 
    const std::vector<std::pair<int, int>>& matches) {
  for (const auto& match : matches) {
    int track_idx = match.first;
    int det_idx = match.second;
    
    tracks_[track_idx]->update(detections[det_idx]);
    
    // Update history
    if (track_history_.find(tracks_[track_idx]->id) == track_history_.end()) {
      track_history_[tracks_[track_idx]->id] = {};
    }
    track_history_[tracks_[track_idx]->id].push_back(detections[det_idx]->center);
  }
}

void MultiObjectTracker::markMissedTracks(const std::vector<int>& unmatched_track_indices) {
  for (int track_idx : unmatched_track_indices) {
    tracks_[track_idx]->markMissed();
  }
}

void MultiObjectTracker::deleteOldTracks() {
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(),
      [](const std::shared_ptr<Track>& track) {
        return track->state == TrackState::DELETED;
      }),
    tracks_.end());
}

}  // namespace awsim_object_tracker