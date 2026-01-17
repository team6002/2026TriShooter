package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConfig {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeFollowerConfig = new SparkMaxConfig();

    static {
        intakeConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kBrake)
                .inverted(IntakeConstants.kInverted)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        intakeConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        intakeConfig
                .closedLoop
                .pid(
                        IntakeConstants.kP,
                        IntakeConstants.kI,
                        IntakeConstants.kD)
                .outputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        intakeConfig.closedLoop.feedForward.svag(
                IntakeConstants.kS, 
                IntakeConstants.kV, 
                IntakeConstants.kA, 
                IntakeConstants.kG);

        intakeFollowerConfig
                .follow(IntakeConstants.kIntakeCanId, true)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        intakeFollowerConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
    }
}
