package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConfig {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    public static final SparkMaxConfig intakeExtenderConfig = new SparkMaxConfig();

    static {
        intakeConfig
            .disableFollowerMode()
            .idleMode(IdleMode.kBrake)
            .inverted(IntakeConstants.kInverted)
            .smartCurrentLimit(40)
            .voltageCompensation(12.0);
        intakeConfig
            .encoder
            .positionConversionFactor((Math.PI * 2) / IntakeConstants.kGearRatio)
            .velocityConversionFactor((Math.PI * 2) / (60 * IntakeConstants.kGearRatio))
            .quadratureAverageDepth(2)
            .quadratureMeasurementPeriod(10);
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
        
        intakeExtenderConfig
            .disableFollowerMode()
            .idleMode(IdleMode.kBrake)
            .inverted(ExtenderConstants.kInverted)
            .smartCurrentLimit(10)
            .voltageCompensation(12.0);
        intakeExtenderConfig.absoluteEncoder
            .positionConversionFactor(Math.PI * 2)
            .velocityConversionFactor((Math.PI * 2) / 60)
            .averageDepth(2);
        intakeExtenderConfig.closedLoop.maxMotion
            .cruiseVelocity(ExtenderConstants.kMaxVel)
            .maxAcceleration(ExtenderConstants.kMaxAccel); 
        intakeExtenderConfig.closedLoop
            .pid(
                ExtenderConstants.kP,
                ExtenderConstants.kI,
                ExtenderConstants.kD)
            .outputRange(ExtenderConstants.kMinOutput, ExtenderConstants.kMaxOutput)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        intakeExtenderConfig.closedLoop.feedForward
            .svag(
                ExtenderConstants.kS,
                ExtenderConstants.kV,
                ExtenderConstants.kA,
                ExtenderConstants.kG);
    }
}
