package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingParams;
import frc.robot.utils.constants.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class CMD_ShootFuelSim extends Command {
  private final AbstractDriveTrainSimulation driveSim;
  private final Intake intake; // Added intake instance
  private int timer;
  private int shooterIndex = 0;

  // State tracking to prevent Auto-skipping
  private boolean seenAnyBalls = false;
  private int emptyHopperCounter = 0;
  private double startTime;

  private static final Translation2d CENTER_SHOOTER_OFFSET =
      new Translation2d(Units.inchesToMeters(7), 0);
  private static final Translation2d LEFT_SHOOTER_OFFSET =
      new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(6));
  private static final Translation2d RIGHT_SHOOTER_OFFSET =
      new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(-6));
  private static final Translation2d[] SHOOTER_OFFSETS = {
    CENTER_SHOOTER_OFFSET, LEFT_SHOOTER_OFFSET, RIGHT_SHOOTER_OFFSET
  };

  /**
   * @param driveSim The simulation of the drivetrain to shoot from.
   * @param intake The specific intake subsystem instance to pull fuel from.
   */
  public CMD_ShootFuelSim(AbstractDriveTrainSimulation driveSim, Intake intake) {
    this.driveSim = driveSim;
    this.intake = intake;
    addRequirements(intake); // Ensure no other command uses this intake simultaneously
  }

  @Override
  public void initialize() {
    timer = 0;
    seenAnyBalls = false;
    emptyHopperCounter = 0;
    startTime = Timer.getFPGATimestamp();

    if (!RobotBase.isSimulation()) return;

    Logger.recordOutput("Commands/CMD_ShootFuelSim", true);
    System.out.println("[CMD_ShootFuelSim] Started. Initial Hopper: " + intake.getHopperCount());
  }

  @Override
  public void execute() {
    if (!RobotBase.isSimulation()) return;

    int currentBallCount = intake.getHopperCount(); // Use instance instead of static

    // Logic to handle simulation delay/initialization
    if (currentBallCount > 0) {
      seenAnyBalls = true;
      emptyHopperCounter = 0;
    } else {
      emptyHopperCounter++;
    }

    // Shooting Loop
    if (currentBallCount > 0) {
      if (timer >= 2) { // 0.2s between shots
        fireProjectile();
        timer = 0;
      } else {
        timer++;
      }
    } else {
      timer++;
    }
  }

  private void fireProjectile() {
    Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();
    Translation2d shooterOffset = SHOOTER_OFFSETS[shooterIndex];

    ShootingParams params =
        calculateLeadingParams(robotPose, shooterOffset, FieldConstants.getHubPose());

    // Update Simulation State via instance
    intake.removeFuel();

    // Spawn the physical projectile
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                robotPose.getTranslation(),
                shooterOffset,
                driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                robotPose.getRotation(),
                Inches.of(21),
                MetersPerSecond.of(params.shooterReference()),
                Radians.of(params.hoodReference())));

    Logger.recordOutput("Sim/LaunchVelocityMPS", params.shooterReference());
    Logger.recordOutput("Sim/LaunchAngleDegrees", Math.toDegrees(params.hoodReference()));

    shooterIndex = (shooterIndex + 1) % 3;
  }

  @Override
  public boolean isFinished() {
    double elapsed = Timer.getFPGATimestamp() - startTime;

    // Timeout if we wait too long without seeing balls
    if (!seenAnyBalls && elapsed > 2.5) return true;

    // Finish when balls were seen and are now gone for 0.3s
    return seenAnyBalls && emptyHopperCounter > 15;
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Commands/CMD_ShootFuelSim", false);
  }

  private ShootingParams calculateLeadingParams(
      Pose2d robotPose, Translation2d shooterOffset, Translation2d hubPosition) {
    Translation2d shooterOffsetField = shooterOffset.rotateBy(robotPose.getRotation());
    Translation2d shooterPos = robotPose.getTranslation().plus(shooterOffsetField);

    ChassisSpeeds speeds = driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    Translation2d relativeVel = robotVel.unaryMinus();

    double distance = shooterPos.getDistance(hubPosition);
    ShootingParams params = ShooterConstants.getSimShootingParams(distance);

    Translation2d targetPos = hubPosition;

    for (int i = 0; i < 5; i++) {
      double predictedDistance = shooterPos.getDistance(targetPos);
      params = ShooterConstants.getSimShootingParams(predictedDistance);
      double projSpeedHorizontal = Math.cos(params.hoodReference()) * params.shooterReference();

      if (projSpeedHorizontal > 0.5) {
        double t = predictedDistance / projSpeedHorizontal;
        targetPos = hubPosition.plus(relativeVel.times(t));
      }
    }
    return params;
  }
}
