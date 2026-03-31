package frc.robot.subsystems.opprobots;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class AIRobotInSimulation extends SubsystemBase {
  /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
  public static final Pose2d[] ROBOT_QUEENING_POSITIONS =
      new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(3.55, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(3.55, 8, new Rotation2d())
      };

  public final SelfControlledSwerveDriveSimulation driveSimulation;
  private final Pose2d queeningPose;

  @SuppressWarnings("unused")
  private final int id;

  public Intake intake;

  public AIRobotInSimulation(int id) {
    this.id = id;
    this.queeningPose = ROBOT_QUEENING_POSITIONS[id];

    // 1. Create the drive simulation FIRST
    this.driveSimulation =
        new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(DriveConstants.mapleSimConfig, queeningPose));

    // 2. Create the Intake using the drive simulation we just made
    // No more passing 'null'!
    this.intake = new Intake(new IntakeIOSim(driveSimulation.getDriveTrainSimulation()));

    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
  }

  // PathPlanner configuration
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          55, // Robot mass in kg
          8, // Robot MOI
          new ModuleConfig(
              Units.inchesToMeters(2),
              3.5,
              1.2,
              DCMotor.getFalcon500(1).withReduction(8.14),
              60,
              1), // Swerve module config
          new Translation2d(0.6, 0.6),
          new Translation2d(-0.6, 0.6),
          new Translation2d(-0.6, -0.6),
          new Translation2d(0.6, -0.6) // Track length and width
          );

  // PathPlanner PID settings
  private final PPHolonomicDriveController driveController =
      new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));

  /** Follow path command for opponent robots */
  public Command opponentRobotFollowPath(PathPlannerPath path) {
    return new FollowPathCommand(
        path, // Specify the path
        // Provide actual robot pose in simulation, bypassing odometry error
        driveSimulation::getActualPoseInSimulationWorld,
        // Provide actual robot speed in simulation, bypassing encoder measurement error
        driveSimulation::getActualSpeedsRobotRelative,
        // Chassis speeds output
        (speeds, feedforwards) ->
            driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
        driveController, // Specify PID controller
        PP_CONFIG, // Specify robot configuration
        // Flip path based on alliance side
        () ->
            DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Red),
        this // AIRobotInSimulation is a subsystem; this command should use it as a requirement
        );
  }

  public static final AIRobotInSimulation[] instances =
      new AIRobotInSimulation[5]; // you can create as many opponent robots as you needs

  public static void startOpponentRobotSimulations() {
    try {
      for (int i = 0; i < 5; i++) {
        // The constructor now handles everything (Drive + Intake) safely
        instances[i] = new AIRobotInSimulation(i);

        Logger.recordOutput(
            "OppRobot/Pose" + i,
            instances[i].driveSimulation.getDriveTrainSimulation().getSimulatedDriveTrainPose());
      }
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load opponent robot simulations: " + e.getMessage(), e.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < instances.length; i++) {
      // Only log if this specific AI robot was actually created
      if (instances[i] != null && instances[i].driveSimulation != null) {
        Logger.recordOutput(
            "OppRobot/Pose" + i,
            instances[i].driveSimulation.getDriveTrainSimulation().getSimulatedDriveTrainPose());
      }
    }
  }
}
