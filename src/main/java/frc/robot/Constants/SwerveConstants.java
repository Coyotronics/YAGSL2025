package frc.robot.Constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module.
    public static final boolean TURNING_ENCODER_INVERTED = true;

    // Free speed of the driving motor in rotations per second
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = 5676 / 60;

    // Diameter of the wheel in meters
    public static final double WHEEL_DIAMETER_METERS = 0.0762;

    // Circumference of the wheel in meters
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    // Gear reduction ratio for the driving motor
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);

    // Free speed of the drive wheel in rotations per second
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
            / DRIVING_MOTOR_REDUCTION;

    // Conversion factor for driving encoder position to meters
    public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
            / DRIVING_MOTOR_REDUCTION;

    // Conversion factor for driving encoder velocity to meters per second
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
            / DRIVING_MOTOR_REDUCTION) / 60.0;

    // Conversion factor for turning encoder position to radians
    public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI);

    // Conversion factor for turning encoder velocity to radians per second
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0;

    // Minimum input for turning encoder position PID in radians
    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0;

    // Maximum input for turning encoder position PID in radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR;

    // PID constants for driving motor
    public static final double DRIVING_P = 0.04;
    public static final double DRIVING_I = 0;
    public static final double DRIVING_D = 0;
    public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    public static final double DRIVING_MIN_OUTPUT = -1;
    public static final double DRIVING_MAX_OUTPUT = 1;

    // PID constants for turning motor
    public static final double TURNING_P = 1;
    public static final double TURNING_I = 0;
    public static final double TURNING_D = 0;
    public static final double TURNING_FF = 0;
    public static final double TURNING_MIN_OUTPUT = -1;
    public static final double TURNING_MAX_OUTPUT = 1;

    // Idle mode for driving motor
    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    // Idle mode for turning motor
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    // Current limit for driving motor in amps
    public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50;

    // Current limit for turning motor in amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 20;
}
