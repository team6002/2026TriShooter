package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterConfig {
    public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
    public static final SparkMaxConfig leftShooterConfig = new SparkMaxConfig();
    public static final SparkMaxConfig rightShooterConfig = new SparkMaxConfig();

    static {
        shooterConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kCoast)
                .inverted(ShooterConstants.kInverted)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        shooterConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        shooterConfig
                .closedLoop
                .pid(
                        ShooterConstants.kP,
                        ShooterConstants.kI,
                        ShooterConstants.kD)
                .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        shooterConfig.closedLoop.feedForward.svag(
                ShooterConstants.kS,
                ShooterConstants.kV,
                ShooterConstants.kA,
                ShooterConstants.kG);

        leftShooterConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kCoast)
                .inverted(ShooterConstants.kInverted)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        leftShooterConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        leftShooterConfig
                .closedLoop
                .pid(
                        ShooterConstants.kP,
                        ShooterConstants.kI,
                        ShooterConstants.kD)
                .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        leftShooterConfig.closedLoop.feedForward.svag(
                ShooterConstants.kS,
                ShooterConstants.kV,
                ShooterConstants.kA,
                ShooterConstants.kG);

        rightShooterConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kCoast)
                .inverted(ShooterConstants.kInverted)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        rightShooterConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        rightShooterConfig
                .closedLoop
                .pid(
                        ShooterConstants.kP,
                        ShooterConstants.kI,
                        ShooterConstants.kD)
                .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rightShooterConfig.closedLoop.feedForward.svag(
                ShooterConstants.kS,
                ShooterConstants.kV,
                ShooterConstants.kA,
                ShooterConstants.kG);
    }
}
