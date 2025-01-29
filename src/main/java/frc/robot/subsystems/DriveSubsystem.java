// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Measurements;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule front_left = new MAXSwerveModule(
            Measurements.FRONT_LEFT_DRIVING_CAN_ID,
            Measurements.FRONT_LEFT_TURNING_CAN_ID,
            Measurements.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule front_right = new MAXSwerveModule(
            Measurements.FRONT_RIGHT_DRIVING_CAN_ID,
            Measurements.FRONT_RIGHT_TURNING_CAN_ID,
            Measurements.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule back_left = new MAXSwerveModule(
            Measurements.BACK_LEFT_DRIVING_CAN_ID,
            Measurements.BACK_LEFT_TURNING_CAN_ID,
            Measurements.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule back_right = new MAXSwerveModule(
            Measurements.BACK_RIGHT_DRIVING_CAN_ID,
            Measurements.BACK_RIGHT_TURNING_CAN_ID,
            Measurements.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

    // The gyro sensor
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    // Slew rate filter variables for controlling lateral acceleration
    private double current_rotation = 0.0;
    private double current_translation_dir = 0.0;
    private double current_translation_mag = 0.0;

    private SlewRateLimiter mag_limiter = new SlewRateLimiter(Measurements.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter rot_limiter = new SlewRateLimiter(Measurements.ROTATIONAL_SLEW_RATE);
    private double prev_time = WPIUtilJNI.now() * 1e-6;

    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            Measurements.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(getAngle()),
            new SwerveModulePosition[] {
                    front_left.getPosition(),
                    front_right.getPosition(),
                    back_left.getPosition(),
                    back_right.getPosition()
            });

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Angle", getAngle());

        // Update the odometry in the periodic block
        odometry.update(
                Rotation2d.fromDegrees(getAngle()),
                new SwerveModulePosition[] {
                        front_left.getPosition(),
                        front_right.getPosition(),
                        back_left.getPosition(),
                        back_right.getPosition()
                });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                Rotation2d.fromDegrees(getAngle()),
                new SwerveModulePosition[] {
                        front_left.getPosition(),
                        front_right.getPosition(),
                        back_left.getPosition(),
                        back_right.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param x_speed        Speed of the robot in the x direction (forward).
     * @param y_speed        Speed of the robot in the y direction (sideways).
     * @param rotation           Angular rate of the robot.
     * @param field_relative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rate_limit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double x_speed, double y_speed, double rotation, boolean field_relative, boolean rate_limit) {

        double x_speed_commanded;
        double y_speed_commanded;
        SmartDashboard.putNumber("XSpeed", x_speed);
        SmartDashboard.putNumber("YSpeed", y_speed);
        SmartDashboard.putNumber("Rotation", rotation);

        if (rate_limit) {
            // Convert XY to polar for rate limiting
            double input_translation_dir = Math.atan2(y_speed, x_speed);
            double input_translation_mag = Math.sqrt(Math.pow(x_speed, 2) + Math.pow(y_speed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double direction_slew_rate;
            if (current_translation_mag != 0.0) {
                direction_slew_rate = Math.abs(Measurements.DIRECTION_SLEW_RATE / current_translation_mag);
            } else {
                direction_slew_rate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double current_time = WPIUtilJNI.now() * 1e-6;
            double elapsed_time = current_time - prev_time;
            double angle_dif = SwerveUtils.AngleDifference(input_translation_dir, current_translation_dir);
            if (angle_dif < 0.45 * Math.PI) {
                current_translation_dir = SwerveUtils.StepTowardsCircular(current_translation_dir,
                        input_translation_dir,
                        direction_slew_rate * elapsed_time);
                current_translation_mag = mag_limiter.calculate(input_translation_mag);
            } else if (angle_dif > 0.85 * Math.PI) {
                if (current_translation_mag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
                    // keep currentTranslationDir unchanged
                    current_translation_mag = mag_limiter.calculate(0.0);
                } else {
                    current_translation_dir = SwerveUtils.WrapAngle(current_translation_dir + Math.PI);
                    current_translation_mag = mag_limiter.calculate(input_translation_mag);
                }
            } else {
                current_translation_dir = SwerveUtils.StepTowardsCircular(current_translation_dir,
                        input_translation_dir,
                        direction_slew_rate * elapsed_time);
                current_translation_mag = mag_limiter.calculate(0.0);
            }
            prev_time = current_time;

            x_speed_commanded = current_translation_mag * Math.cos(current_translation_dir);
            y_speed_commanded = current_translation_mag * Math.sin(current_translation_dir);
            current_rotation = rot_limiter.calculate(rotation);

        } else {
            x_speed_commanded = x_speed;
            y_speed_commanded = y_speed;
            current_rotation = rotation;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double x_speed_delivered = x_speed_commanded * Measurements.MAX_SPEED_METERS_PER_SECOND;
        double y_speed_delivered = y_speed_commanded * Measurements.MAX_SPEED_METERS_PER_SECOND;
        double rot_delivered = current_rotation * Measurements.MAX_ANGULAR_SPEED;

        var swerve_module_states = Measurements.DRIVE_KINEMATICS.toSwerveModuleStates(
                field_relative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered,
                                Rotation2d.fromDegrees(getAngle()))
                        : new ChassisSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerve_module_states, Measurements.MAX_SPEED_METERS_PER_SECOND);
        front_left.setDesiredState(swerve_module_states[0]);
        front_right.setDesiredState(swerve_module_states[1]);
        back_left.setDesiredState(swerve_module_states[2]);
        back_right.setDesiredState(swerve_module_states[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        front_left.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        front_right.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        back_left.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        back_right.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desired_states The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desired_states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desired_states, Measurements.MAX_SPEED_METERS_PER_SECOND);
        front_left.setDesiredState(desired_states[0]);
        front_right.setDesiredState(desired_states[1]);
        back_left.setDesiredState(desired_states[2]);
        back_right.setDesiredState(desired_states[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        front_left.resetEncoders();
        back_left.resetEncoders();
        front_right.resetEncoders();
        back_right.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(getAngle()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (Measurements.GYRO_REVERSED ? -1.0 : 1.0);
    }

    public double getAngle() {
        return -(gyro.getAngle());
    }
}
