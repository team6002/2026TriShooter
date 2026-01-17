package frc.robot.subsystems.climb;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbConfig {
    public static final SparkMaxConfig climbConfig = new SparkMaxConfig();

    static {
        climbConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kCoast)
                .inverted(ClimbConstants.kInverted)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        climbConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        climbConfig
                .closedLoop
                .pid(
                        ClimbConstants.kP,
                        ClimbConstants.kI,
                        ClimbConstants.kD)
                .outputRange(ClimbConstants.kMinOutput, ClimbConstants.kMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        climbConfig.closedLoop.feedForward.svag(
                ClimbConstants.kS,
                ClimbConstants.kV,
                ClimbConstants.kA,
                ClimbConstants.kG);
    }
}
