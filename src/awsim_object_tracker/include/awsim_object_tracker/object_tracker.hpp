#ifndef AWSIM_OBJECT_TRACKER_OBJECT_TRACKER_HPP
#define AWSIM_OBJECT_TRACKER_OBJECT_TRACKER_HPP

#include <vector>
#include <memory>
#include <map>
#include <string>
#include <chrono>

#include <Eigen/Dense>

#include "awsim_object_tracker/kalman_filter.hpp"
#include "awsim_object_tracker/perception_pipeline.hpp"

namespace awsim_object_tracker {

/**
 * @brief Track state enumeration
 */
enum class TrackState {
  TENTATIVE = 0,
  CONFIRMED = 1,
  DELETED = 2
};

/**
 * @brief Convert TrackState to string
 * 
 * @param state Track state
 * @return std::string String representation
 */
inline std::string trackStateToString(TrackState state) {
  switch (state) {
    case TrackState::TENTATIVE: return "TENTATIVE";
    case TrackState::CONFIRMED: return "CONFIRMED";
    case TrackState::DELETED: return "DELETED";
    default: return "UNKNOWN";
  }
}

/**
 * @brief Represents a single object track
 */
struct Track {
  int id;                                  // Unique track ID
  std::shared_ptr<KalmanFilter> kf;        // Kalman filter for state estimation
  TrackState state;                        // Track state
  int hits;                                // Number of successful updates
  int time_since_update;                   // Frames since last update
  int age;                                 // Total age of the track
  std::shared_ptr<Detection> last_detection;  // Last associated detection
  std::chrono::steady_clock::time_point creation_time;  // Track creation time
  Eigen::Quaterniond orientation;          // Current orientation estimate
  Eigen::Vector3d angular_velocity;        // Angular velocity estimate

  /**
   * @brief Construct a new Track object
   * 
   * @param id Track ID
   * @param kf Kalman filter
   * @param state Initial state
   * @param hits Initial hit count
   * @param time_since_update Initial time since update
   * @param age Initial age
   * @param detection Initial detection
   */
  Track(int id, 
        std::shared_ptr<KalmanFilter> kf, 
        TrackState state,
        int hits,
        int time_since_update,
        int age,
        std::shared_ptr<Detection> detection = nullptr);

  /**
   * @brief Get current position estimate
   * 
   * @return Position vector [x, y, z]
   */
  Eigen::Vector3d getPosition() const;

  /**
   * @brief Get current velocity estimate
   * 
   * @return Velocity vector [vx, vy, vz]
   */
  Eigen::Vector3d getVelocity() const;

  /**
   * @brief Get current orientation estimate
   * 
   * @return Orientation quaternion
   */
  Eigen::Quaterniond getOrientation() const;

  /**
   * @brief Update orientation with smoothing
   * 
   * @param new_orientation New orientation from detection
   * @param smoothing_factor Smoothing factor (0.0 = no update, 1.0 = full update)
   */
  void updateOrientation(const Eigen::Quaterniond& new_orientation, double smoothing_factor = 0.3);

  /**
   * @brief Get current angular velocity estimate
   * 
   * @return Angular velocity vector [wx, wy, wz]
   */
  Eigen::Vector3d getAngularVelocity() const;

  /**
   * @brief Predict next state
   */
  void predict();

  /**
   * @brief Update with detection
   * 
   * @param detection Detection to update with
   */
  void update(const std::shared_ptr<Detection>& detection);

  /**
   * @brief Mark track as missed (no detection associated)
   */
  void markMissed();
};

/**
 * @brief Multi-Object Tracker using Kalman Filter and Hungarian Algorithm Association
 */
class MultiObjectTracker {
public:
  /**
   * @brief Construct a new Multi Object Tracker object
   * 
   * @param max_distance Maximum distance for data association (meters)
   * @param dt Time step between frames (seconds)
   */
  MultiObjectTracker(double max_distance = 5.0, double dt = 0.1);

  /**
   * @brief Update tracker with new detections
   * 
   * @param detections List of detections for current frame
   */
  void update(const std::vector<std::shared_ptr<Detection>>& detections);

  /**
   * @brief Get all active (confirmed) tracks
   * 
   * @return std::vector<std::shared_ptr<Track>> List of active tracks
   */
  std::vector<std::shared_ptr<Track>> getActiveTracks() const;

  /**
   * @brief Get all tracks (including tentative)
   * 
   * @return std::vector<std::shared_ptr<Track>> List of all tracks
   */
  std::vector<std::shared_ptr<Track>> getAllTracks() const;

  /**
   * @brief Get current positions of all tracks
   * 
   * @return std::map<int, Eigen::Vector3d> Map of track ID to position
   */
  std::map<int, Eigen::Vector3d> getTrackPositions() const;

  /**
   * @brief Get position history for a specific track
   * 
   * @param track_id Track ID
   * @return std::vector<Eigen::Vector3d> Position history
   */
  std::vector<Eigen::Vector3d> getTrackHistory(int track_id) const;

private:
  double max_distance_;  // Maximum distance for data association
  double dt_;            // Time step
  std::vector<std::shared_ptr<Track>> tracks_;  // List of tracks
  int track_id_counter_;  // Counter for generating unique track IDs
  int frame_count_;       // Frame counter
  
  // Tracking history for visualization
  std::map<int, std::vector<Eigen::Vector3d>> track_history_;

  /**
   * @brief Predict all tracks to current frame
   */
  void predict();

  /**
   * @brief Calculate cost matrix for data association
   * 
   * @param detections List of detections
   * @return Eigen::MatrixXd Cost matrix [tracks x detections]
   */
  Eigen::MatrixXd calculateCostMatrix(const std::vector<std::shared_ptr<Detection>>& detections) const;

  /**
   * @brief Associate detections to tracks using the Hungarian algorithm
   * 
   * @param detections List of detections
   * @return std::tuple with (matches, unmatched_detections, unmatched_tracks)
   */
  std::tuple<std::vector<std::pair<int, int>>, std::vector<int>, std::vector<int>> 
  associateDetectionsToTracks(const std::vector<std::shared_ptr<Detection>>& detections);

  /**
   * @brief Create new tracks for unmatched detections
   * 
   * @param detections List of all detections
   * @param unmatched_detection_indices Indices of unmatched detections
   */
  void initiateNewTracks(const std::vector<std::shared_ptr<Detection>>& detections, 
                         const std::vector<int>& unmatched_detection_indices);

  /**
   * @brief Update matched tracks with detections
   * 
   * @param detections List of detections
   * @param matches List of (track_idx, detection_idx) pairs
   */
  void updateTracks(const std::vector<std::shared_ptr<Detection>>& detections, 
                   const std::vector<std::pair<int, int>>& matches);

  /**
   * @brief Mark unmatched tracks as missed
   * 
   * @param unmatched_track_indices Indices of unmatched tracks
   */
  void markMissedTracks(const std::vector<int>& unmatched_track_indices);

  /**
   * @brief Remove tracks marked for deletion
   */
  void deleteOldTracks();
};

}  // namespace awsim_object_tracker

#endif  // AWSIM_OBJECT_TRACKER_OBJECT_TRACKER_HPP
