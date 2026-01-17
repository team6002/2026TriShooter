package frc.robot.subsystems.conveyor;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ConveyorConfig {
    public static final SparkMaxConfig conveyorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig conveyorFollowerConfig = new SparkMaxConfig();

    static {
        conveyorConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kBrake)
                .inverted(ConveyorConstants.kInverted)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        conveyorConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        conveyorConfig
                .closedLoop
                .pid(
                        ConveyorConstants.kP,
                        ConveyorConstants.kI,
                        ConveyorConstants.kD)
                .outputRange(ConveyorConstants.kMinOutput, ConveyorConstants.kMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        conveyorConfig.closedLoop.feedForward.svag(
                ConveyorConstants.kS,
                ConveyorConstants.kV,
                ConveyorConstants.kA,
                ConveyorConstants.kG);
        
        conveyorFollowerConfig
                .follow(ConveyorConstants.kConveyorCanId, true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
    }
}
