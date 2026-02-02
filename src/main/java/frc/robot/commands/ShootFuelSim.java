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
import frc.robot.subsystems.shooter.ShooterConstants.ShootingParams;
import frc.robot.constants.FieldConstants;

public class ShootFuelSim extends Command {
    private final AbstractDriveTrainSimulation driveSim;
    private int timer;

    private static final Translation2d CENTER_SHOOTER_OFFSET = new Translation2d(Units.inchesToMeters(5.5), 0);
    private static final Translation2d LEFT_SHOOTER_OFFSET = new Translation2d(Units.inchesToMeters(5.5), Units.inchesToMeters(6));
    private static final Translation2d RIGHT_SHOOTER_OFFSET = new Translation2d(Units.inchesToMeters(5.5), Units.inchesToMeters(-6));
    private static final Translation2d[] SHOOTER_OFFSETS = {
        CENTER_SHOOTER_OFFSET,
        LEFT_SHOOTER_OFFSET,
        RIGHT_SHOOTER_OFFSET
    };

    public ShootFuelSim(AbstractDriveTrainSimulation driveSim){
        this.driveSim = driveSim;
    }

    @Override
    public void initialize(){
        timer = 0;
        if(!RobotBase.isSimulation()) return;
    }

    @Override
    public void execute(){
        if (timer >= 2 && IntakeIOSim.numObjectsInHopper() > 0) {
            Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();

            double distance = robotPose.getTranslation().getDistance(FieldConstants.getHubPose());
            ShootingParams params = ShooterConstants.getShootingParams(distance);

            IntakeIOSim.obtainFuelFromHopper();

            SimulatedArena.getInstance().addGamePieceProjectile(
                new RebuiltFuelOnFly(
                    robotPose.getTranslation(),
                    SHOOTER_OFFSETS[(int)(Math.random()*3)],
                    driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    robotPose.getRotation(),
                    Inches.of(21),
                    MetersPerSecond.of(params.velocityMPS()),
                    Radians.of(params.angRad())
                )
            );

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