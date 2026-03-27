package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.CMD_ShootFuelSim;
import frc.robot.subsystems.opprobots.AIRobotInSimulation;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_9470auto implements Auto {
  @SuppressWarnings("static-access")
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(
                    () ->
                        AIRobotInSimulation.instances[0].driveSimulation.setSimulationWorldPose(
                            new Pose2d(12.1, 7.5, new Rotation2d()))),
                AIRobotInSimulation.instances[0].opponentRobotFollowPath(
                    PathPlannerPath.fromPathFile("opprobot"))
                // ,new
                // InstantCommand(()->AIRobotInSimulation.instances[1].driveSimulation.setSimulationWorldPose(new Pose2d(12.1,0.6, new Rotation2d())))
                // ,AIRobotInSimulation.instances[1].opponentRobotFollowPath(PathPlannerPath.fromPathFile("opprobot2"))
                ),
            new SequentialCommandGroup(
                new InstantCommand(
                    () ->
                        AIRobotInSimulation.instances[1].driveSimulation.setSimulationWorldPose(
                            new Pose2d(12.1, 0.6, new Rotation2d()))),
                AIRobotInSimulation.instances[1].opponentRobotFollowPath(
                    PathPlannerPath.fromPathFile("opprobot2"))),
            new SequentialCommandGroup(
                new InstantCommand(
                    () ->
                        AIRobotInSimulation.instances[2].driveSimulation.setSimulationWorldPose(
                            new Pose2d(4.453, 8.1 - 7.444, new Rotation2d()))),
                AIRobotInSimulation.instances[2].opponentRobotFollowPathFlipped(
                    PathPlannerPath.fromPathFile("swipemiddle9470")),
                AIRobotInSimulation.instances[2].opponentRobotFollowPathFlipped(
                    PathPlannerPath.fromPathFile("shootfirst9470")),
                new CMD_ShootFuelSim(
                    AIRobotInSimulation.instances[2].driveSimulation.getDriveTrainSimulation(),
                    AIRobotInSimulation.instances[2].intake),
                AIRobotInSimulation.instances[2].opponentRobotFollowPathFlipped(
                    PathPlannerPath.fromPathFile("secondsweep9470")),
                AIRobotInSimulation.instances[2].opponentRobotFollowPathFlipped(
                    PathPlannerPath.fromPathFile("shootsecond9470")),
                new CMD_ShootFuelSim(
                    AIRobotInSimulation.instances[2].driveSimulation.getDriveTrainSimulation(),
                    AIRobotInSimulation.instances[2].intake)),
            new SequentialCommandGroup(
                new InstantCommand(
                    () ->
                        AIRobotInSimulation.instances[3].driveSimulation.setSimulationWorldPose(
                            new Pose2d(13.1, 2.5, new Rotation2d().fromDegrees(-45)))),
                AIRobotInSimulation.instances[3].opponentRobotFollowPath(
                    PathPlannerPath.fromPathFile("opprobot3"))),
            new SequentialCommandGroup(
                new InstantCommand(
                    () ->
                        AIRobotInSimulation.instances[4].driveSimulation.setSimulationWorldPose(
                            new Pose2d(3.55, 4, new Rotation2d()))),
                AIRobotInSimulation.instances[4].opponentRobotFollowPath(
                    PathPlannerPath.fromPathFile("partnerrobot1")),
                new WaitCommand(2),
                new InstantCommand(
                    () -> AIRobotInSimulation.instances[4].intake.addFuelToHopper(24)),
                AIRobotInSimulation.instances[4].opponentRobotFollowPath(
                    PathPlannerPath.fromPathFile("partnerrobot1shoot")),
                new CMD_ShootFuelSim(
                    AIRobotInSimulation.instances[4].driveSimulation.getDriveTrainSimulation(),
                    AIRobotInSimulation.instances[4].intake)),
            new SequentialCommandGroup(
                setAutoStartPose("swipemiddle9470", false, robot.drive),
                followPath("swipemiddle9470", false),
                followPath("shootfirst9470", false),
                new CMD_ShootFuelSim(robot.driveSimulation, robot.intake),
                followPath("secondsweep9470", false),
                followPath("shootsecond9470", false),
                new CMD_ShootFuelSim(robot.driveSimulation, robot.intake))));
  }
}
