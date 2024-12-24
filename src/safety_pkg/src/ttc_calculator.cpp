#include "safety_pkg/ttc_calculator.hpp"
#include <cmath>
#include <limits>

// Constructor that initializes the TTC calculator with a threshold value
// This threshold determines when we consider a collision risk to be critical
TTCCalculator::TTCCalculator(double ttc_threshold)
    : ttc_threshold_(ttc_threshold) {}

// Calculates the minimum Time To Collision (TTC) from all laser scan beams
// Parameters:
// - scan_msg: Contains array of range measurements from the LIDAR sensor
// - current_velocity: The vehicle's current velocity
double TTCCalculator::calculateMinTTC(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg,
    double current_velocity) {
    
    // If we're not moving, there's no risk of collision
    // Return infinity to indicate no collision risk
    if (current_velocity <= 0) {
        return std::numeric_limits<double>::infinity();
    }

    // Initialize minimum TTC to infinity - we'll find the smallest value
    double min_ttc = std::numeric_limits<double>::infinity();
    // Start with the minimum angle of the laser scan
    double angle = scan_msg->angle_min;

    // Iterate through all range measurements in the scan
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        // Get the range measurement for this beam
        double range = scan_msg->ranges[i];
        
        // Only process valid range measurements
        // Invalid measurements (inf/nan) are skipped
        if (std::isfinite(range)) {
            // Calculate TTC for this specific beam
            double ttc = calculateTTC(range, angle, current_velocity);
            // Keep track of the minimum TTC found
            min_ttc = std::min(min_ttc, ttc);
        }
        
        // Move to next angle using the scan's angular resolution
        angle += scan_msg->angle_increment;
    }

    return min_ttc;
}

// Calculates TTC for a single laser beam
// Parameters:
// - range: Distance to obstacle for this beam
// - angle: Angle of the laser beam relative to vehicle's forward direction
// - velocity: Current velocity of the vehicle
double TTCCalculator::calculateTTC(double range, double angle, double velocity) {
    // Project vehicle's velocity onto the beam direction
    // Negative because we're interested in approaching obstacles
    // cos(angle) gives the component of velocity in the beam direction
    double range_rate = -velocity * std::cos(angle);
    
    // Only consider beams where we're getting closer to obstacles
    // range_rate < 0 means we're approaching the obstacle
    if (range_rate < 0) {
        // TTC = distance / closing_speed
        // We use -range_rate because we want a positive TTC
        return range / (-range_rate);
    }
    
    // If we're not approaching the obstacle, return infinity
    // This means no collision risk in this direction
    return std::numeric_limits<double>::infinity();
}