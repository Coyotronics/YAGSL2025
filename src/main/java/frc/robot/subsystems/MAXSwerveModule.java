package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.SparkMaxConfigs;

public class MAXSwerveModule {
    private final SparkMax driving_spark_max;
    private final SparkMax turning_spark_max;

    private final RelativeEncoder driving_encoder;
    private final AbsoluteEncoder turning_encoder;

    private final SparkClosedLoopController driving_pid_controller;
    private final SparkClosedLoopController turning_pid_controller;

    private double chassis_angular_offset = 0;
    private SwerveModuleState desired_state = new SwerveModuleState(0.0, new Rotation2d());

    public MAXSwerveModule(int driving_can_id, int turning_can_id, double chassis_angular_offset) {
        driving_spark_max = new SparkMax(driving_can_id, MotorType.kBrushless);
        turning_spark_max = new SparkMax(turning_can_id, MotorType.kBrushless);

        driving_encoder = driving_spark_max.getEncoder();
        turning_encoder = turning_spark_max.getAbsoluteEncoder();

        driving_pid_controller = driving_spark_max.getClosedLoopController();
        turning_pid_controller = turning_spark_max.getClosedLoopController();


        driving_spark_max.configure(SparkMaxConfigs.get_driving_config(), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        turning_spark_max.configure(SparkMaxConfigs.get_turning_config(), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        this.chassis_angular_offset = chassis_angular_offset;
        desired_state.angle = new Rotation2d(turning_encoder.getPosition());
        driving_encoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driving_encoder.getVelocity(),
                new Rotation2d(turning_encoder.getPosition() - chassis_angular_offset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driving_encoder.getPosition(),
                new Rotation2d(turning_encoder.getPosition() - chassis_angular_offset));
    }

    public void setDesiredState(SwerveModuleState desired_state) {
        SwerveModuleState corrected_desired_state = new SwerveModuleState();
        corrected_desired_state.speedMetersPerSecond = desired_state.speedMetersPerSecond;
        corrected_desired_state.angle = desired_state.angle
                .plus(Rotation2d.fromRadians(chassis_angular_offset));

        corrected_desired_state.optimize(new Rotation2d(turning_encoder.getPosition()));

        driving_pid_controller.setReference(corrected_desired_state.speedMetersPerSecond,
                ControlType.kVelocity);
        turning_pid_controller.setReference(corrected_desired_state.angle.getRadians(),
                ControlType.kPosition);

        this.desired_state = desired_state;
    }

    public void resetEncoders() {
        driving_encoder.setPosition(0);
    }
}
