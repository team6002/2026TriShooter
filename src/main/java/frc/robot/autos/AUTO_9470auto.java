package frc.robot.autos;

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

public class AUTO_9470auto implements AutoWithRobotSimulation {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        new ParallelCommandGroup(
            // Instance 0
            new SequentialCommandGroup(
                AutoWithRobotSimulation.setOppStartPose(new Pose2d(12.1, 7.5, new Rotation2d()), 0),
                AutoWithRobotSimulation.oppRobotFollowPath("opprobot", 0, false)),

            // Instance 1
            new SequentialCommandGroup(
                AutoWithRobotSimulation.setOppStartPose(new Pose2d(12.1, 0.6, new Rotation2d()), 1),
                AutoWithRobotSimulation.oppRobotFollowPath("opprobot2", 1, false)),

            // Instance 2
            new SequentialCommandGroup(
                AutoWithRobotSimulation.setOppStartPose(
                    new Pose2d(4.453, 0.656, new Rotation2d()), 2),
                AutoWithRobotSimulation.oppRobotFollowPath("swipemiddle9470", 2, true),
                AutoWithRobotSimulation.oppRobotFollowPath("shootfirst9470", 2, true),
                AutoWithRobotSimulation.shootFuelOppRobot(2),
                AutoWithRobotSimulation.oppRobotFollowPath("secondsweep9470", 2, true),
                AutoWithRobotSimulation.oppRobotFollowPath("shootsecond9470", 2, true),
                AutoWithRobotSimulation.shootFuelOppRobot(2)),

            // Instance 3
            new SequentialCommandGroup(
                AutoWithRobotSimulation.setOppStartPose(
                    new Pose2d(13.1, 2.5, Rotation2d.fromDegrees(-45)), 3),
                AutoWithRobotSimulation.oppRobotFollowPath("opprobot3", 3, true)),

            // Instance 4
            new SequentialCommandGroup(
                AutoWithRobotSimulation.setOppStartPose(new Pose2d(3.55, 4, new Rotation2d()), 4),
                AutoWithRobotSimulation.oppRobotFollowPath("partnerrobot1", 4, false),
                new WaitCommand(2),
                new InstantCommand(
                    () -> AIRobotInSimulation.instances[4].intake.addFuelToHopper(24)),
                AutoWithRobotSimulation.oppRobotFollowPath("partnerrobot1shoot", 4, false),
                AutoWithRobotSimulation.shootFuelOppRobot(4)),

            // Player robot
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
