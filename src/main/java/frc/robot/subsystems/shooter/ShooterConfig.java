package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterConfig {
    public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();

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
    }
}
