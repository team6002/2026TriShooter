package frc.robot.subsystems.superstructure.elevator;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ElevatorProperties {
    public String name;
    public int primaryCanID;
    public boolean primaryInverted;
    public boolean hasFollowerMotor;
    public int followerCanID;
    public boolean followerInverted;
    public Distance kOffsetDistance;
    public LinearVelocity kMaxVelocity;
    public LinearAcceleration kMaxAcceleration;

    public Distance kTolerance;
    public double minOutputVoltage = -1;
    public double maxOutputVoltage = 1;
    public double kP = 1.0, kI = 0.0, kD = 0.0;
    public double kS = 0.0, kG = 0.0, kV = 0.0, kA = 0.0;
    public double positionConversionFactor = (2 * Math.PI);

    public ElevatorProperties() {}
    ;

    public ElevatorFeedforward getArmFeedforward() {
        return new ElevatorFeedforward(kS, kG, kV, kA);
    }

    public SparkMaxConfig getPrimaryConfig() {
        SparkMaxConfig _config = new SparkMaxConfig();

        _config.idleMode(IdleMode.kBrake)
                .inverted(primaryInverted)
                .voltageCompensation(12.0)
                .disableFollowerMode()
                .smartCurrentLimit(50);
        _config.closedLoop
                .pid(kP, kI, kD)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(minOutputVoltage, maxOutputVoltage);
        _config.encoder
                .uvwAverageDepth(2)
                .uvwMeasurementPeriod(10)
                .positionConversionFactor(positionConversionFactor)
                .velocityConversionFactor(positionConversionFactor / 60);

        _config.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);

        return _config;
    }
    ;

    public SparkMaxConfig getFollowerConfig() {
        SparkMaxConfig _config = new SparkMaxConfig();
        _config.idleMode(IdleMode.kBrake)
                .inverted(followerInverted)
                .voltageCompensation(12.0)
                .follow(primaryCanID, true)
                .smartCurrentLimit(50);
        _config.closedLoop
                .pid(kP, kI, kD)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(minOutputVoltage, maxOutputVoltage);
        _config.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);

        return _config;
    }
    ;
}
