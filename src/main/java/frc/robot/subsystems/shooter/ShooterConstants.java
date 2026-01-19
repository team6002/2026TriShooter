package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final int kShooterCanId = 9;
    public static final int kLeftShooterCanId = 10;
    public static final int kRightShooterCanId = 11;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;

    public static final double kV = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kA = 0.0;

    public static final double kPSim = 0.0;
    public static final double kISim = 0.0;
    public static final double kDSim = 0.0;
    public static final double kFFSim = 0.0;

    public static final double kSSim = 0.0;
    public static final double kVSim = 0.0;
    public static final double kGSim = 0.0;
    public static final double kASim = 0.0;

    public static final boolean kInverted = true;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;

    public static final double kShoot = 4500;
    public static final double kOff = 0;
    public static final double kHolding = 2;

    public static final double kGearRatio = 1;

    private static final double[][] SHOOTING_TABLE = {
        {1.00, 77.0, 6.43},
        {2.00, 72.6, 7.24},
        {3.00, 65.0, 7.65},
        {4.00, 65.1, 8.47},
        {5.00, 45.6, 9.08},
        {6.00, 46.9, 9.49},
        {7.00, 39.8, 10.51}
    };

    public static final record ShootingParams(double angRad, double velocityMPS) {}

    public static final ShootingParams getShootingParams(double distance) {
        if (distance <= SHOOTING_TABLE[0][0]) {
            return new ShootingParams(SHOOTING_TABLE[0][1], SHOOTING_TABLE[0][2]);
        }
        if (distance >= SHOOTING_TABLE[SHOOTING_TABLE.length - 1][0]) {
            int last = SHOOTING_TABLE.length - 1;
            return new ShootingParams(SHOOTING_TABLE[last][1], SHOOTING_TABLE[last][2]);
        }

        for (int i = 0; i < SHOOTING_TABLE.length - 1; i++) {
            if (distance >= SHOOTING_TABLE[i][0] && distance <= SHOOTING_TABLE[i + 1][0]) {
                double d0 = SHOOTING_TABLE[i][0];
                double d1 = SHOOTING_TABLE[i + 1][0];
                double t = (distance - d0) / (d1 - d0);

                double angle = SHOOTING_TABLE[i][1] + t * (SHOOTING_TABLE[i + 1][1] - SHOOTING_TABLE[i][1]);
                double velocity = SHOOTING_TABLE[i][2] + t * (SHOOTING_TABLE[i + 1][2] - SHOOTING_TABLE[i][2]);

                return new ShootingParams(Units.degreesToRadians(angle), velocity);
            }
        }

        return new ShootingParams(Units.degreesToRadians(75), 7.0);
    }
}
