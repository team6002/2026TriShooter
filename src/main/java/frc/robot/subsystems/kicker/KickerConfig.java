package frc.robot.subsystems.kicker;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class KickerConfig {
    public static final SparkMaxConfig kickerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig kickerFollowerConfig = new SparkMaxConfig();

    static {
        kickerConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kBrake)
                .inverted(KickerConstants.kInverted)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        kickerConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        kickerConfig
                .closedLoop
                .pid(
                        KickerConstants.kP,
                        KickerConstants.kI,
                        KickerConstants.kD)
                .outputRange(KickerConstants.kMinOutput, KickerConstants.kMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        kickerConfig.closedLoop.feedForward.svag(
                KickerConstants.kS,
                KickerConstants.kV,
                KickerConstants.kA,
                KickerConstants.kG);

        kickerFollowerConfig
                .follow(KickerConstants.kKickerCanId, true)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        kickerFollowerConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
    }
}
