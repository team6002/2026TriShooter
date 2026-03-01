package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.utils.constants.RobotMode;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_DoubleTrench implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot, boolean mirrored)
      throws IOException, ParseException {
    return Commands.sequence(
        // reset odometry and put intake down
        setAutoStartPose("SwipeHalfMiddleTrench", mirrored, robot.drive),
        new ParallelCommandGroup(
            Robot.CURRENT_ROBOT_MODE == RobotMode.REAL
                ? new CMD_Intake(robot.intake)
                : Commands.none(),
            // run out and intake half of our side of the field
            followPath("SwipeHalfMiddleTrench", mirrored)),
        followPath("ShootDoubleTrench", mirrored),
        // shoot fuel
        Robot.CURRENT_ROBOT_MODE == RobotMode.REAL
            ? new CMD_Shoot(
                    robot.conveyor,
                    robot.hood,
                    robot.intake,
                    robot.kicker,
                    robot.shooter,
                    0.2,
                    Math.toRadians(20000))
                .withTimeout(2)
            : new ShootFuelSim(robot.driveSimulation, robot.hood, robot.shooter),
        new ParallelCommandGroup(
            // Robot.CURRENT_ROBOT_MODE == RobotMode.REAL
            //     ? new CMD_Intake(robot.intake)
            //     : Commands.none(),
            // run out and intake half of our side of the field
            followPath("SwipeHalfMiddleDoubleTrench", mirrored)),
        followPath("ShootDoubleTrench", mirrored),
        // shoot fuel
        Robot.CURRENT_ROBOT_MODE == RobotMode.REAL
            ? new CMD_Shoot(
                    robot.conveyor,
                    robot.hood,
                    robot.intake,
                    robot.kicker,
                    robot.shooter,
                    0.2,
                    Math.toRadians(20000))
                .withTimeout(2)
            : new ShootFuelSim(robot.driveSimulation, robot.hood, robot.shooter));
  }
}
