package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
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
        shooterConfig.encoder
            .positionConversionFactor(Math.PI * 2)
            .velocityConversionFactor((Math.PI * 2) / 60)
            .quadratureAverageDepth(2)
            .quadratureMeasurementPeriod(5);
        shooterConfig
            .closedLoop
            .pid(
                ShooterConstants.kLoadPID[1][0],
                ShooterConstants.kLoadPID[1][1],
                ShooterConstants.kLoadPID[1][2]
            )
            .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);


        leftShooterConfig
            .disableFollowerMode()
            .idleMode(IdleMode.kCoast)
            .inverted(ShooterConstants.kInverted)
            .smartCurrentLimit(40)
            .voltageCompensation(12.0);
        leftShooterConfig.encoder
            .positionConversionFactor(Math.PI * 2)
            .velocityConversionFactor((Math.PI * 2) / 60)
            .quadratureAverageDepth(2)
            .quadratureMeasurementPeriod(5);
        leftShooterConfig
            .closedLoop
            .pid(
                ShooterConstants.kLoadPID[0][0],
                ShooterConstants.kLoadPID[0][1],
                ShooterConstants.kLoadPID[0][2],
                ClosedLoopSlot.kSlot0
            )
            .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);


        rightShooterConfig
            .disableFollowerMode()
            .idleMode(IdleMode.kCoast)
            .inverted(ShooterConstants.kInverted)
            .smartCurrentLimit(40)
            .voltageCompensation(12.0);
        rightShooterConfig.encoder
            .positionConversionFactor(Math.PI * 2)
            .velocityConversionFactor((Math.PI * 2) / 60)
            .quadratureAverageDepth(2)
            .quadratureMeasurementPeriod(5);
        rightShooterConfig
            .closedLoop
            .pid(
                ShooterConstants.kLoadPID[2][0],
                ShooterConstants.kLoadPID[2][1],
                ShooterConstants.kLoadPID[2][2]
            )
            .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }
}
