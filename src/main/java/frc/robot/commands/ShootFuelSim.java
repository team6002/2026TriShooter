package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.constants.FieldConstants;

public class ShootFuelSim extends Command {
    private final AbstractDriveTrainSimulation driveSim;
    private int timer;
    private int shooterIndex = 0;

    private static final Translation2d CENTER_SHOOTER_OFFSET =
        new Translation2d(Units.inchesToMeters(-12), 0);
    private static final Translation2d LEFT_SHOOTER_OFFSET =
        new Translation2d(Units.inchesToMeters(-12), Units.inchesToMeters(6));
    private static final Translation2d RIGHT_SHOOTER_OFFSET =
        new Translation2d(Units.inchesToMeters(-12), Units.inchesToMeters(-6));

    public ShootFuelSim(AbstractDriveTrainSimulation driveSim){
        this.driveSim = driveSim;
    }

    @Override
    public void initialize(){
        timer = 0;
        shooterIndex = 0;

        if(!RobotBase.isSimulation()) return;
    }

    @Override
    public void execute(){
        if (timer > 3 && IntakeIOSim.numObjectsInHopper() > 0) {
            Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();

            double distance = robotPose.getTranslation().getDistance(FieldConstants.HubPose);
            ShooterConstants.ShootingParams params = ShooterConstants.getShootingParams(distance);

            IntakeIOSim.obtainFuelFromHopper();

            Translation2d shooterOffset;
            switch (shooterIndex) {
                case 0: shooterOffset = CENTER_SHOOTER_OFFSET; break;
                case 1: shooterOffset = LEFT_SHOOTER_OFFSET; break;
                default: shooterOffset = RIGHT_SHOOTER_OFFSET; break;
            }

            SimulatedArena.getInstance().addGamePieceProjectile(
                new RebuiltFuelOnFly(
                    robotPose.getTranslation(),
                    shooterOffset,
                    driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    robotPose.getRotation(),
                    Inches.of(6),
                    MetersPerSecond.of(params.velocityMPS()),
                    Radians.of(params.angRad())
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
}