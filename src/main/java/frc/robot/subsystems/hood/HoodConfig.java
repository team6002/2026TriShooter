package frc.robot.subsystems.hood;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HoodConfig {
    public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

    static {
        hoodConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kCoast)
                .inverted(HoodConstants.kInverted)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        hoodConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        hoodConfig
                .closedLoop
                .pid(
                        HoodConstants.kP,
                        HoodConstants.kI,
                        HoodConstants.kD)
                .outputRange(HoodConstants.kMinOutput, HoodConstants.kMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        hoodConfig.closedLoop.feedForward.svag(
                HoodConstants.kS,
                HoodConstants.kV,
                HoodConstants.kA,
                HoodConstants.kG);
    }
}
