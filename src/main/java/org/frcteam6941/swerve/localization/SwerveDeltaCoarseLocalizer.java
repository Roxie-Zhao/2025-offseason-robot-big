package org.frcteam6941.swerve.localization;
 
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
 
import frc.robot.RobotConstants;
 
import lombok.Synchronized;
 
import org.frcteam6941.utils.InterpolatingTreeMap;
import org.frcteam6941.utils.MovingAveragePose2d;
 
// This class implements a coarse localization system for a swerve drive robot
// It uses odometry and pose estimation to track the robot's position and velocity
public class SwerveDeltaCoarseLocalizer implements Localizer {
    private final Object statusLock = new Object();
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final int poseBufferSize;
    private final int velocityBufferSize;
    private final int accelerationBufferSize;
    private Pose2d previousPose = null;
    private Pose2d previousVelocity = new Pose2d();
    private double distanceDriven = 0.0;
    private InterpolatingTreeMap<Double, Pose2d> fieldToVehicle;
    private Pose2d vehicleVelocityMeasured;
    private MovingAveragePose2d vehicleVelocityMeasuredFilter;
    private Pose2d vehicleAccelerationMeasured;
    private MovingAveragePose2d vehicleAccelerationMeasuredFilter;
    private Pose2d vehicleVelocityPredicted;
    private MovingAveragePose2d vehicleVelocityPredictedFilter;
 
    // Constructor initializes the localization system with given parameters and initial module positions
    public SwerveDeltaCoarseLocalizer(SwerveDriveKinematics kinematics, int poseBufferSize, int velocityBufferSize,
                                      int accelerationBufferSize, SwerveModulePosition[] initPosition) {
        this.poseBufferSize = poseBufferSize;
        this.velocityBufferSize = velocityBufferSize;
        this.accelerationBufferSize = accelerationBufferSize;
        fieldToVehicle = new InterpolatingTreeMap<Double, Pose2d>(poseBufferSize);
        vehicleVelocityMeasured = new Pose2d();
        vehicleVelocityMeasuredFilter = new MovingAveragePose2d(velocityBufferSize);
        vehicleAccelerationMeasured = new Pose2d();
        vehicleAccelerationMeasuredFilter = new MovingAveragePose2d(accelerationBufferSize);
        vehicleVelocityPredicted = new Pose2d();
        vehicleVelocityPredictedFilter = new MovingAveragePose2d(velocityBufferSize);
 
        vehicleVelocityMeasuredFilter.add(new Pose2d());
        vehicleAccelerationMeasuredFilter.add(new Pose2d());
 
        swerveOdometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), initPosition);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), initPosition, new Pose2d());

    }
 
    // Updates the localization state with the current time, time delta, gyro angle, and module positions
    public synchronized Pose2d updateWithTime(double time, double dt, Rotation2d gyroAngle,
                                              SwerveModulePosition[] moduleStates) {
        dt = RobotConstants.LOOPER_DT;
        synchronized (statusLock) {
            // Get pose from kinematics update
            Pose2d pose = swerveOdometry.update(gyroAngle, moduleStates);
            poseEstimator.updateWithTime(time, gyroAngle, moduleStates);
 
            // First, get the displacement
            if (previousPose == null) {
                previousPose = pose;
            }
            Pose2d poseDelta = pose.relativeTo(previousPose);
 
            distanceDriven += poseDelta.getTranslation().getNorm();
            fieldToVehicle.put(time, new Pose2d(
                    fieldToVehicle.lastEntry().getValue().getTranslation().plus(poseDelta.getTranslation()),
                    fieldToVehicle.lastEntry().getValue().getRotation().rotateBy(poseDelta.getRotation())));
 
            vehicleVelocityMeasured = new Pose2d(poseDelta.getX() / dt, poseDelta.getY() / dt,
                    poseDelta.getRotation().times(1.0 / dt));
            vehicleVelocityMeasuredFilter.add(vehicleVelocityMeasured);
 
 
            // Second, get the acceleration
            if (previousVelocity == null) {
                previousVelocity = vehicleVelocityMeasured;
            }
 
            Pose2d velocityDelta = vehicleVelocityMeasured.relativeTo(previousVelocity);
            vehicleAccelerationMeasured = new Pose2d(velocityDelta.getX() / dt, velocityDelta.getY() / dt,
                    velocityDelta.getRotation().times(1.0 / dt));
            vehicleAccelerationMeasuredFilter.add(vehicleAccelerationMeasured);
 
            // Third, update prediction using acceleration
            vehicleVelocityPredicted = vehicleVelocityMeasuredFilter.getAverage()
                    .plus(new Transform2d(vehicleAccelerationMeasured.getTranslation(),
                            vehicleAccelerationMeasured.getRotation()).times(dt));
            vehicleVelocityPredictedFilter.add(vehicleVelocityPredicted);
 
            // Finally, update system state and ready for the next iteration
            previousPose = pose;
            previousVelocity = vehicleVelocityMeasured;
 
 
            // Return pose
            return pose;
        }
    }
 
    // Returns the latest pose estimated by the swerve drive odometry
    @Override
    @Synchronized
    public synchronized Pose2d getLatestPose() {
        synchronized (statusLock) {
            return swerveOdometry.getPoseMeters();
        }
    }
 
    // Returns the coarse field pose estimated by the pose estimator
    @Override
    @Synchronized
    public synchronized Pose2d getCoarseFieldPose(double time) {
        synchronized (statusLock) {
            return poseEstimator.getEstimatedPosition();
        }
    }
 
    // Returns the measured velocity of the vehicle
    @Override
    @Synchronized
    public synchronized Pose2d getMeasuredVelocity() {
        synchronized (statusLock) {
            return vehicleVelocityMeasured;
        }
    }
 
    // Returns the predicted velocity of the vehicle
    @Override
    @Synchronized
    public synchronized Pose2d getPredictedVelocity() {
        synchronized (statusLock) {
            return vehicleVelocityPredicted;
        }
    }
 
    // Returns the smoothed predicted velocity of the vehicle
    @Override
    @Synchronized
    public synchronized Pose2d getSmoothedPredictedVelocity() {
        synchronized (statusLock) {
            return vehicleVelocityPredictedFilter.getAverage();
        }
    }
 
    // Returns the predicted pose of the vehicle at a given lookahead time
    @Override
    public synchronized Pose2d getPredictedPose(double lookahead) {
        synchronized (statusLock) {
            return getLatestPose().transformBy(new Transform2d(getSmoothedPredictedVelocity().getTranslation(),
                    getSmoothedPredictedVelocity().getRotation()).times(-lookahead));
        }
    }
 
    // Returns the measured acceleration of the vehicle
    @Override
    @Synchronized
    public synchronized Pose2d getMeasuredAcceleration() {
        synchronized (statusLock) {
            return vehicleAccelerationMeasured;
        }
    }
 
    // Returns the smoothed measured velocity of the vehicle
    @Override
    public synchronized Pose2d getSmoothedVelocity() {
        synchronized (statusLock) {
            return vehicleVelocityMeasuredFilter.getAverage();
        }
    }
 
    // Returns the smoothed measured acceleration of the vehicle
    public synchronized Pose2d getSmoothedAccleration() {
        synchronized (statusLock) {
            return vehicleAccelerationMeasuredFilter.getAverage();
        }
    }
 
    // Returns the interpolated pose at a given time
    @Override
    public synchronized Pose2d getPoseAtTime(double time) {
        synchronized (statusLock) {
            return fieldToVehicle.getInterpolated(time, 0.5);
        }
    }
 
    // Returns the total distance driven by the vehicle
    @Override
    public synchronized double getDistanceDriven() {
        return distanceDriven;
    }
 
    // Adds a vision measurement to the pose estimator
    @Override
    public synchronized void addMeasurement(double time, Pose2d measuredPose, Pose2d stdDeviation) {
        synchronized (statusLock) {
            poseEstimator.addVisionMeasurement(
                    measuredPose,
                    time,
                    MatBuilder.fill(Nat.N3(), Nat.N1(), stdDeviation.getX(), stdDeviation.getY(), stdDeviation.getRotation().getDegrees())
            );
        }
    }
 
    // Adds a vision measurement to the pose estimator with a precomputed standard deviation matrix
    @Override
    public synchronized void addMeasurement(double time, Pose2d measuredPose, Matrix<N3, N1> stdDeviation) {
        synchronized (statusLock) {
            poseEstimator.addVisionMeasurement(
                    measuredPose,
                    time,
                    stdDeviation
            );
        }
    }
 
    // Resets the localization system to a new pose and module positions
    public synchronized void reset(Pose2d resetPose, SwerveModulePosition[] modulePositions) {
        synchronized (statusLock) {
            swerveOdometry.resetPosition(resetPose.getRotation(), modulePositions, resetPose);
            poseEstimator.resetPosition(resetPose.getRotation(), modulePositions, resetPose);
            previousPose = null;
            fieldToVehicle = new InterpolatingTreeMap<Double, Pose2d>(poseBufferSize);
            fieldToVehicle.put(Timer.getFPGATimestamp(), resetPose);
            vehicleVelocityMeasured = new Pose2d();
            vehicleVelocityMeasuredFilter = new MovingAveragePose2d(velocityBufferSize);
            vehicleAccelerationMeasured = new Pose2d();
            vehicleAccelerationMeasuredFilter = new MovingAveragePose2d(accelerationBufferSize);
            vehicleVelocityPredictedFilter = new MovingAveragePose2d(velocityBufferSize);
 
            vehicleVelocityMeasuredFilter.add(new Pose2d());
            vehicleAccelerationMeasuredFilter.add(new Pose2d());
        }
    }
}