package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeIOSim;

public class ShootFuelSim extends Command {
    private final AbstractDriveTrainSimulation driveSim;
    private int timer;
    private int shooterIndex = 0;

    private static final double[][] SHOOTING_TABLE = {
        {1.50, 75, 7.00},
        {2.50, 72, 7.75},
        {3.50, 69, 8.25},
        {4.50, 66, 9},
        {5.50, 63, 9.75}
    };

    private static final Translation2d CENTER_SHOOTER_OFFSET =
        new Translation2d(Units.inchesToMeters(-12), 0);
    private static final Translation2d LEFT_SHOOTER_OFFSET =
        new Translation2d(Units.inchesToMeters(-12), Units.inchesToMeters(6));
    private static final Translation2d RIGHT_SHOOTER_OFFSET =
        new Translation2d(Units.inchesToMeters(-12), Units.inchesToMeters(-6));

    private final Pose2d BlueHubPose =
        new Pose2d(new Translation2d(Units.inchesToMeters(182), Units.inchesToMeters(159)), null);
    private final Pose2d RedHubPose =
        new Pose2d(new Translation2d(Units.inchesToMeters(469), Units.inchesToMeters(159)), null);

    public ShootFuelSim(AbstractDriveTrainSimulation driveSim){
        this.driveSim = driveSim;
    }

    @Override
    public void initialize(){
        timer = 0;
        shooterIndex = 0;
    }

    @Override
    public void execute(){
        if (timer > 3 && IntakeIOSim.numObjectsInHopper() > 0) {

            Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();
            Pose2d hubPose =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? BlueHubPose : RedHubPose;

            double distance = robotPose.getTranslation().getDistance(hubPose.getTranslation());
            ShootingParams params = getShootingParams(distance);

            IntakeIOSim.obtainFuelFromHopper();

            Translation2d shooterOffset;
            switch (shooterIndex) {
                case 0: shooterOffset = CENTER_SHOOTER_OFFSET; break;
                case 1: shooterOffset = LEFT_SHOOTER_OFFSET; break;
                default: shooterOffset = RIGHT_SHOOTER_OFFSET; break;
            }

            SimulatedArena.getInstance().addGamePieceProjectile(
                new ReefscapeAlgaeOnFly(
                    robotPose.getTranslation(),
                    shooterOffset,
                    driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    robotPose.getRotation(),
                    Inches.of(6),
                    MetersPerSecond.of(params.velocityMPS),
                    Degrees.of(params.angleDegrees)
                )
            );

            shooterIndex = (shooterIndex + 1) % 3;
            timer = 0;

        } else {
            timer++;
        }
    }

    @Override
    public boolean isFinished(){
        return IntakeIOSim.numObjectsInHopper() <= 0;
    }

    private record ShootingParams(double angleDegrees, double velocityMPS) {}

    private ShootingParams getShootingParams(double distance) {
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

                return new ShootingParams(angle, velocity);
            }
        }

        return new ShootingParams(75.0, 7.0);
    }
}
