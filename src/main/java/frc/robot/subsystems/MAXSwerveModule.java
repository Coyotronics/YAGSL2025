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
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants.SwerveConstants;

public class MAXSwerveModule {
    private final SparkMax driving_spark_max;
    private final SparkMax turning_spark_max;

    private final SparkMaxConfig driving_spark_max_config;
    private final SparkMaxConfig turning_spark_max_config;

    private final RelativeEncoder driving_encoder;
    private final AbsoluteEncoder turning_encoder;

    private final SparkClosedLoopController driving_pid_controller;
    private final SparkClosedLoopController turning_pid_controller;

    private double chassis_angular_offset = 0;
    private SwerveModuleState desired_state = new SwerveModuleState(0.0, new Rotation2d());

    public MAXSwerveModule(int driving_can_id, int turning_can_id, double chassis_angular_offset) {
        driving_spark_max = new SparkMax(driving_can_id, MotorType.kBrushless);
        turning_spark_max = new SparkMax(turning_can_id, MotorType.kBrushless);

        driving_spark_max_config = new SparkMaxConfig();
        turning_spark_max_config = new SparkMaxConfig();

        driving_encoder = driving_spark_max.getEncoder();
        turning_encoder = turning_spark_max.getAbsoluteEncoder();
        driving_pid_controller = driving_spark_max.getClosedLoopController();
        turning_pid_controller = turning_spark_max.getClosedLoopController();

        // driving motor configuration
        driving_spark_max_config.idleMode(SwerveConstants.DRIVING_MOTOR_IDLE_MODE).smartCurrentLimit(
                SwerveConstants.DRIVING_MOTOR_CURRENT_LIMIT);

        // driving motor encoder configuration
        driving_spark_max_config.encoder
                .positionConversionFactor(SwerveConstants.DRIVING_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(SwerveConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        // driving motor PID controller configuration
        driving_spark_max_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(
                SwerveConstants.DRIVING_P, SwerveConstants.DRIVING_I, SwerveConstants.DRIVING_D,
                SwerveConstants.DRIVING_FF)
                .outputRange(SwerveConstants.DRIVING_MIN_OUTPUT, SwerveConstants.DRIVING_MAX_OUTPUT);

        // turning motor configuration
        turning_spark_max_config.idleMode(SwerveConstants.TURNING_MOTOR_IDLE_MODE).smartCurrentLimit(
                SwerveConstants.TURNING_MOTOR_CURRENT_LIMIT);

        // turning motor encoder configuration
        turning_spark_max_config.encoder
                .positionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(SwerveConstants.TURNING_ENCODER_VELOCITY_FACTOR).inverted(
                        SwerveConstants.TURNING_ENCODER_INVERTED);

        // turning motor PID controller configuration
        turning_spark_max_config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pidf(
                SwerveConstants.TURNING_P, SwerveConstants.TURNING_I, SwerveConstants.TURNING_D,
                SwerveConstants.TURNING_FF)
                .outputRange(SwerveConstants.TURNING_MIN_OUTPUT, SwerveConstants.TURNING_MAX_OUTPUT)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(SwerveConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
                        SwerveConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        driving_spark_max.configure(driving_spark_max_config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        turning_spark_max.configure(turning_spark_max_config, ResetMode.kResetSafeParameters,
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
