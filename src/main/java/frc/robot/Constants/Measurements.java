package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Measurements {
    // Driving Parameters - Note that these are not the maximum capable speeds of the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8; // 4.8 is 80% of allowed speed
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // Maximum angular speed in radians per second

    public static final double DIRECTION_SLEW_RATE = 1.2; // Direction slew rate in radians per second
    public static final double MAGNITUDE_SLEW_RATE = 1.8; // Magnitude slew rate in percent per second (1 = 100%)
    public static final double ROTATIONAL_SLEW_RATE = 2.0; // Rotational slew rate in percent per second (1 = 100%)

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(26.5); // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(26.5); // Distance between front and back wheels on robot

    // Swerve drive kinematics configuration
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // SPARK MAX CAN IDs for driving motors
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 45;
    public static final int BACK_LEFT_DRIVING_CAN_ID = 12;
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 20;
    public static final int BACK_RIGHT_DRIVING_CAN_ID = 10;

    // SPARK MAX CAN IDs for turning motors
    public static final int FRONT_LEFT_TURNING_CAN_ID = 23;
    public static final int BACK_LEFT_TURNING_CAN_ID = 13;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 21;
    public static final int BACK_RIGHT_TURNING_CAN_ID = 22;

    // Gyro configuration
    public static final boolean GYRO_REVERSED = false;
}
