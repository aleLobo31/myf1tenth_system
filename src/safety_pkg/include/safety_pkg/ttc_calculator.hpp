#ifndef TTC_CALCULATOR_HPP  // Header guard to prevent multiple inclusion
#define TTC_CALCULATOR_HPP

// Include necessary ROS2 message type for laser scan data
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

/**
 * @brief Class for calculating Time To Collision (TTC) using laser scan data
 * 
 * This class provides functionality to calculate the minimum time to collision
 * based on laser scan measurements and the current velocity of the vehicle.
 */
class TTCCalculator {
public:
    /**
     * @brief Constructor that takes a TTC threshold value
     * @param ttc_threshold The minimum acceptable time to collision (in seconds)
     */
    explicit TTCCalculator(double ttc_threshold);
    
    /**
     * @brief Calculates the minimum TTC from all laser scan beams
     * @param scan_msg Pointer to laser scan message containing range data
     * @param current_velocity Current velocity of the vehicle
     * @return Minimum TTC value across all valid laser beams
     */
    double calculateMinTTC(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg,
                          double current_velocity);

    /**
     * @brief Getter for the TTC threshold value
     * @return The current TTC threshold
     */
    double getThreshold() const { return ttc_threshold_; }

private:
    double ttc_threshold_;  // Stores the minimum acceptable time to collision

    /**
     * @brief Calculates TTC for a single laser beam
     * @param range Distance to obstacle for this beam
     * @param angle Angle of the laser beam
     * @param velocity Current velocity of the vehicle
     * @return TTC value for this specific beam
     */
    double calculateTTC(double range, double angle, double velocity);
};

#endif  // TTC_CALCULATOR_HPP