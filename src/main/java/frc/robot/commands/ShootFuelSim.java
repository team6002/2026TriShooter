package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingParams;
import frc.robot.utils.constants.FieldConstants;

public class ShootFuelSim extends Command {
    private final AbstractDriveTrainSimulation driveSim;
    private final Hood hood;
    private final Shooter shooter;
    private int timer;

    private static final Translation2d CENTER_SHOOTER_OFFSET = new Translation2d(Units.inchesToMeters(7), 0);
    private static final Translation2d LEFT_SHOOTER_OFFSET = new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(6));
    private static final Translation2d RIGHT_SHOOTER_OFFSET = new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(-6));
    private static final Translation2d[] SHOOTER_OFFSETS = {
        CENTER_SHOOTER_OFFSET,
        LEFT_SHOOTER_OFFSET,
        RIGHT_SHOOTER_OFFSET
    };

    public ShootFuelSim(AbstractDriveTrainSimulation driveSim, Hood hood, Shooter shooter){
        this.driveSim = driveSim;
        this.hood = hood;
        this.shooter = shooter;
        addRequirements(shooter); // Ensure we don't conflict with other shooter commands
    }

    @Override
    public void initialize(){
        timer = 0;
        if (!RobotBase.isSimulation()) return;
    }

    private int shooterIndex = 0;

    @Override
    public void execute() {
        // Timer to simulate a 60ms feed delay (3 loops * 20ms)
        Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();
        double distToHubMeters = robotPose.getTranslation().getDistance(FieldConstants.getHubPose());
        ShootingParams shootingParams = ShooterConstants.getShootingParams(distToHubMeters);

        shooter.setReference(shootingParams.shooterReference());
        hood.setReference(shootingParams.hoodReference());

        if (timer >= 3 && IntakeIOSim.numObjectsInHopper() > 0 && shooter.isReady()) {
            Translation2d shooterOffset = SHOOTER_OFFSETS[shooterIndex];

            // 1. Trigger the Physics Simulation "Dip"
            // This calls the spawnBall methods we added to ShooterIOSim
            shooter.spawnSimulatedBall(shooterIndex);

            // 2. Calculate actual exit velocity
            // In a hood shooter, the ball exits at 1/2 the surface velocity of the wheel
            double currentRadPerSec = shooter.getVelocityRadPerSec(shooterIndex);
            double wheelSurfaceSpeed = currentRadPerSec * ShooterConstants.kFlywheelRadiusMeters;
            double exitVelocityMps = wheelSurfaceSpeed / 2.0;

            // 3. Launch the visual projectile
            SimulatedArena.getInstance().addGamePieceProjectile(
                new RebuiltFuelOnFly(
                    robotPose.getTranslation(),
                    shooterOffset,
                    driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    robotPose.getRotation(),
                    Inches.of(21), // Height of the shooter exit
                    MetersPerSecond.of(exitVelocityMps),
                    Degrees.of(hood.getPosition()) // convert hood position in rotations to degrees 85 - hood.getPosition() * 45
                )
            );

            IntakeIOSim.obtainFuelFromHopper();
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