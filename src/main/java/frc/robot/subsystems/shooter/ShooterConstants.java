package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.MapleShooterOptimization;

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

    // Shooting table: {distance (meters), angle (degrees), velocity (m/s), time of flight (s)}
    public static final double[][] SHOOTING_TABLE = {
        {1.0,  83.00, 6.02,  0.7608},
        {2.0,  65.43, 6.43,  0.7482},
        {3.0,  56.90, 7.04,  0.7641},
        {4.0,  58.75, 7.86,  0.9812},
        {5.0,  48.88, 8.47,  0.8978},
        {6.0,  42.88, 9.29,  0.8818},
        {7.0,  41.60, 9.90,  0.9457},
        {8.0,  40.16, 10.51, 0.9960},
        {9.0,  38.67, 11.12, 1.0365},
        {10.0, 37.20, 11.73, 1.0699}
    };

    // Extract columns for MapleShooterOptimization
    private static double[] extractColumn(int col) {
        double[] result = new double[SHOOTING_TABLE.length];
        for (int i = 0; i < SHOOTING_TABLE.length; i++) {
            result[i] = SHOOTING_TABLE[i][col];
        }
        return result;
    }

    public static final MapleShooterOptimization kShooterOptimization = new MapleShooterOptimization(
        "shooterOptimization",
        extractColumn(0), // distance, Meters
        extractColumn(1), // hood angle, Degrees
        extractColumn(2), // shooter velocity, Meters Per Second
        extractColumn(3)  // TOF, Secods
    );

    public static final record ShootingParams(double angRad, double velocityMPS) {}

    public static final ShootingParams getShootingParams(double distance) {
        // Clamp to table bounds
        if (distance <= SHOOTING_TABLE[0][0]) {
            return new ShootingParams(
                Units.degreesToRadians(SHOOTING_TABLE[0][1]), 
                SHOOTING_TABLE[0][2]
            );
        }
        if (distance >= SHOOTING_TABLE[SHOOTING_TABLE.length - 1][0]) {
            int last = SHOOTING_TABLE.length - 1;
            return new ShootingParams(
                Units.degreesToRadians(SHOOTING_TABLE[last][1]), 
                SHOOTING_TABLE[last][2]
            );
        }

        // Linear interpolation
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

        // Fallback (should never reach here)
        return new ShootingParams(Units.degreesToRadians(75), 7.0);
    }
}