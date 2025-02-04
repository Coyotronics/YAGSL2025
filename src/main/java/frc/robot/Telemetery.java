package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Telemetery {
    private static final DoublePublisher module_count_publisher = NetworkTableInstance.getDefault()
            .getTable(
                    "SmartDashboard")
            .getDoubleTopic(
                    "swerve/module_count")
            .publish();

    private static final DoubleArrayPublisher measured_states_publisher = NetworkTableInstance.getDefault()
            .getTable(
                    "SmartDashboard")
            .getDoubleArrayTopic(
                    "swerve/measured_states")
            .publish();

    private static final DoubleArrayPublisher desired_states_publisher = NetworkTableInstance.getDefault()
            .getTable(
                    "SmartDashboard")
            .getDoubleArrayTopic(
                    "swerve/desired_states")
            .publish();

    private static final DoubleArrayPublisher measured_chassis_speeds_publisher = NetworkTableInstance
            .getDefault()
            .getTable(
                    "SmartDashboard")
            .getDoubleArrayTopic(
                    "swerve/measured_chassis_speeds")
            .publish();

    private static final DoubleArrayPublisher desired_chassis_speeds_publisher = NetworkTableInstance.getDefault()
            .getTable(
                    "SmartDashboard")
            .getDoubleArrayTopic(
                    "swerve/desired_chassis_speeds")
            .publish();

    private static final DoublePublisher robot_rotation_publisher = NetworkTableInstance.getDefault()
            .getTable(
                    "SmartDashboard")
            .getDoubleTopic(
                    "swerve/robot_rotation")
            .publish();

    public static SwerveModuleState[] measured_states_obj = new SwerveModuleState[4];

    public static SwerveModuleState[] desired_states_obj = new SwerveModuleState[4];

    public static ChassisSpeeds measured_chassis_speeds_obj = new ChassisSpeeds();

    public static ChassisSpeeds desired_chassis_speeds_obj = new ChassisSpeeds();

    public static Rotation2d robot_rotation_obj = new Rotation2d();

    public static void update() {
        double[] measured_states = new double[4 * 2];
        double[] desired_states = new double[4 * 2];

        for (int i = 0; i < measured_states_obj.length; i++) {
            if (measured_states_obj[i] != null) {
                measured_states[i * 2] = measured_states_obj[i].angle.getDegrees();
                measured_states[i * 2 + 1] = measured_states_obj[i].speedMetersPerSecond;
            }
        }

        for (int i = 0; i < desired_states_obj.length; i++) {
            if (desired_states_obj[i] != null) {
                desired_states[i * 2] = desired_states_obj[i].angle.getDegrees();
                desired_states[i * 2 + 1] = desired_states_obj[i].speedMetersPerSecond;
            }
        }

        measured_states_publisher.set(measured_states);
        desired_states_publisher.set(desired_states);
        robot_rotation_publisher.set(robot_rotation_obj.getDegrees());
    }
}
