package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.RobotMeasurements;
import frc.robot.Constants.SwerveConstants;
import frc.utils.SwerveUtils;

import frc.robot.Telemetery;

public class DriveSubsystem extends SubsystemBase {
    private final MAXSwerveModule front_left = new MAXSwerveModule(
            RobotMeasurements.FRONT_LEFT_DRIVING_CAN_ID,
            RobotMeasurements.FRONT_LEFT_TURNING_CAN_ID,
            RobotMeasurements.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule front_right = new MAXSwerveModule(
            RobotMeasurements.FRONT_RIGHT_DRIVING_CAN_ID,
            RobotMeasurements.FRONT_RIGHT_TURNING_CAN_ID,
            RobotMeasurements.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule back_left = new MAXSwerveModule(
            RobotMeasurements.BACK_LEFT_DRIVING_CAN_ID,
            RobotMeasurements.BACK_LEFT_TURNING_CAN_ID,
            RobotMeasurements.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule back_right = new MAXSwerveModule(
            RobotMeasurements.BACK_RIGHT_DRIVING_CAN_ID,
            RobotMeasurements.BACK_RIGHT_TURNING_CAN_ID,
            RobotMeasurements.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    private double current_rotation = 0.0;
    private double current_translation_dir = 0.0;
    private double current_translation_mag = 0.0;

    private SlewRateLimiter mag_limiter = new SlewRateLimiter(RobotMeasurements.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter rot_limiter = new SlewRateLimiter(RobotMeasurements.ROTATIONAL_SLEW_RATE);
    private double prev_time = WPIUtilJNI.now() * 1e-6;

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            RobotMeasurements.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(get_angle()),
            new SwerveModulePosition[] {
                    front_left.get_position(),
                    front_right.get_position(),
                    back_left.get_position(),
                    back_right.get_position()
            });

    public DriveSubsystem() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::get_pose,
                this::reset_odometry,
                this::get_robot_relative_speeds,
                (speeds, feedforwards) -> drive_robot_relative(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(SwerveConstants.DRIVING_P, SwerveConstants.DRIVING_I,
                                SwerveConstants.DRIVING_D),
                        new PIDConstants(SwerveConstants.TURNING_P, SwerveConstants.TURNING_I,
                                SwerveConstants.TURNING_D)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Angle", get_angle());

        odometry.update(
                Rotation2d.fromDegrees(get_angle()),
                new SwerveModulePosition[] {
                        front_left.get_position(),
                        front_right.get_position(),
                        back_left.get_position(),
                        back_right.get_position()
                });
    }

    public Pose2d get_pose() {
        return odometry.getPoseMeters();
    }

    public void reset_odometry(Pose2d pose) {
        odometry.resetPosition(
                Rotation2d.fromDegrees(get_angle()),
                new SwerveModulePosition[] {
                        front_left.get_position(),
                        front_right.get_position(),
                        back_left.get_position(),
                        back_right.get_position()
                },
                pose);
    }

    public SwerveModuleState[] get_module_states() {
        return new SwerveModuleState[] {
                front_left.get_state(),
                front_right.get_state(),
                back_left.get_state(),
                back_right.get_state()
        };
    }

    public ChassisSpeeds get_robot_relative_speeds() {
        return RobotMeasurements.DRIVE_KINEMATICS.toChassisSpeeds(get_module_states());
    }

    public void drive_robot_relative(ChassisSpeeds speeds) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, true);
    }

    public void drive(double x_speed, double y_speed, double rotation, boolean field_relative, boolean rate_limit) {
        double x_speed_commanded;
        double y_speed_commanded;
        SmartDashboard.putNumber("XSpeed", x_speed);
        SmartDashboard.putNumber("YSpeed", y_speed);
        SmartDashboard.putNumber("Rotation", rotation);

        Telemetery.measured_states_obj[0] = front_left.get_state();
        Telemetery.measured_states_obj[1] = front_right.get_state();
        Telemetery.measured_states_obj[2] = back_left.get_state();
        Telemetery.measured_states_obj[3] = back_right.get_state();

        Telemetery.measured_chassis_speeds_obj = RobotMeasurements.DRIVE_KINEMATICS.toChassisSpeeds(
                new SwerveModuleState[] {
                        front_left.get_state(),
                        front_right.get_state(),
                        back_left.get_state(),
                        back_right.get_state()
                });

        if (rate_limit) {
            double input_translation_dir = Math.atan2(y_speed, x_speed);
            double input_translation_mag = Math.sqrt(Math.pow(x_speed, 2) + Math.pow(y_speed, 2));

            double direction_slew_rate;
            if (current_translation_mag != 0.0) {
                direction_slew_rate = Math.abs(RobotMeasurements.DIRECTION_SLEW_RATE / current_translation_mag);
            } else {
                direction_slew_rate = 500.0;
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
                if (current_translation_mag > 1e-4) {
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

        double x_speed_delivered = x_speed_commanded * RobotMeasurements.MAX_SPEED_METERS_PER_SECOND;
        double y_speed_delivered = y_speed_commanded * RobotMeasurements.MAX_SPEED_METERS_PER_SECOND;
        double rot_delivered = current_rotation * RobotMeasurements.MAX_ANGULAR_SPEED;

        var swerve_module_states = RobotMeasurements.DRIVE_KINEMATICS.toSwerveModuleStates(
                field_relative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered,
                                Rotation2d.fromDegrees(get_angle()))
                        : new ChassisSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerve_module_states, RobotMeasurements.MAX_SPEED_METERS_PER_SECOND);
        front_left.set_desired_state(swerve_module_states[0]);
        front_right.set_desired_state(swerve_module_states[1]);
        back_left.set_desired_state(swerve_module_states[2]);
        back_right.set_desired_state(swerve_module_states[3]);

        Telemetery.desired_states_obj = swerve_module_states.clone();
        Telemetery.desired_chassis_speeds_obj = RobotMeasurements.DRIVE_KINEMATICS
                .toChassisSpeeds(swerve_module_states);
        Telemetery.robot_rotation_obj = Rotation2d.fromDegrees(get_angle());
    }

    public void set_x() {
        front_left.set_desired_state(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        front_right.set_desired_state(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        back_left.set_desired_state(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        back_right.set_desired_state(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void set_module_states(SwerveModuleState[] desired_states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desired_states, RobotMeasurements.MAX_SPEED_METERS_PER_SECOND);
        front_left.set_desired_state(desired_states[0]);
        front_right.set_desired_state(desired_states[1]);
        back_left.set_desired_state(desired_states[2]);
        back_right.set_desired_state(desired_states[3]);
    }

    public void reset_encoders() {
        front_left.reset_encoders();
        back_left.reset_encoders();
        front_right.reset_encoders();
        back_right.reset_encoders();
    }

    public void zero_heading() {
        gyro.reset();
    }

    public double get_heading() {
        return Rotation2d.fromDegrees(get_angle()).getDegrees();
    }

    public double get_turn_rate() {
        return gyro.getRate() * (RobotMeasurements.GYRO_REVERSED ? -1.0 : 1.0);
    }

    public double get_angle() {
        return -(gyro.getAngle());
    }
}
