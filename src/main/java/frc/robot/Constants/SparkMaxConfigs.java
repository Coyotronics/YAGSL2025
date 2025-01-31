package frc.robot.Constants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxConfigs {
    public static SparkMaxConfig get_driving_config() {
        SparkMaxConfig config = new SparkMaxConfig();

        // driving motor configuration
        config
                .idleMode(SwerveConstants.DRIVING_MOTOR_IDLE_MODE)
                .smartCurrentLimit(SwerveConstants.DRIVING_MOTOR_CURRENT_LIMIT);

        // driving motor encoder configuration
        config.encoder
                .positionConversionFactor(SwerveConstants.DRIVING_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(SwerveConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        // driving motor PID controller configuration
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(SwerveConstants.DRIVING_P, SwerveConstants.DRIVING_I, SwerveConstants.DRIVING_D,
                        SwerveConstants.DRIVING_FF)
                .outputRange(SwerveConstants.DRIVING_MIN_OUTPUT, SwerveConstants.DRIVING_MAX_OUTPUT);

        return config;
    }

    public static SparkMaxConfig get_turning_config() {
        SparkMaxConfig config = new SparkMaxConfig();

        // turning motor configuration
        config.idleMode(SwerveConstants.TURNING_MOTOR_IDLE_MODE)
                .smartCurrentLimit(SwerveConstants.TURNING_MOTOR_CURRENT_LIMIT);

        // turning motor encoder configuration
        config.absoluteEncoder
                .positionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(SwerveConstants.TURNING_ENCODER_VELOCITY_FACTOR)
                .inverted(SwerveConstants.TURNING_ENCODER_INVERTED);

        // turning motor PID controller configuration
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(SwerveConstants.TURNING_P, SwerveConstants.TURNING_I, SwerveConstants.TURNING_D,
                        SwerveConstants.TURNING_FF)
                .outputRange(SwerveConstants.TURNING_MIN_OUTPUT, SwerveConstants.TURNING_MAX_OUTPUT)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(SwerveConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
                        SwerveConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        return config;
    }
}
