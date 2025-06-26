package org.frcteam6941.swerve.localization;
 
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
 
// Interface for localizing the robot's position and motion on the field
public interface Localizer {
    // Returns the latest pose of the robot
    Pose2d getLatestPose();
 
    // Returns the pose of the robot at a specific time
    Pose2d getPoseAtTime(double time);
 
    // Returns a predicted pose of the robot a certain time into the future
    Pose2d getPredictedPose(double lookahead);
 
    // Returns a coarse field pose of the robot at a specific time
    Pose2d getCoarseFieldPose(double time);
 
    // Returns the measured velocity of the robot
    Pose2d getMeasuredVelocity();
 
    // Returns the measured acceleration of the robot
    Pose2d getMeasuredAcceleration();
 
    // Returns the predicted velocity of the robot
    Pose2d getPredictedVelocity();
 
    // Returns the smoothed velocity of the robot
    Pose2d getSmoothedVelocity();
 
    // Returns the smoothed predicted velocity of the robot
    Pose2d getSmoothedPredictedVelocity();
 
    // Returns the smoothed acceleration of the robot
    Pose2d getSmoothedAccleration();
 
    // Returns the total distance driven by the robot
    double getDistanceDriven();
 
    // Adds a measurement of the robot's motion to the localization system with a standard deviation
    void addMeasurement(double time, Pose2d measuredDelta, Pose2d stdDeviation);
 
    // Adds a measurement of the robot's motion to the localization system with a standard deviation matrix
    void addMeasurement(double time, Pose2d measuredDelta, Matrix<N3, N1> stdDeviation);
}