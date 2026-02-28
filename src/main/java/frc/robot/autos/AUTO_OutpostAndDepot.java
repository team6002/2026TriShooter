package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.utils.constants.RobotMode;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_OutpostAndDepot implements Auto {

  @Override
  public Command getAutoCommand(RobotContainer robot, boolean mirrored)
      throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("IntakeOutpost", mirrored, robot.drive),
        followPath("IntakeOutpost", mirrored),
        new WaitCommand(1),
        followPath("ShootOutpostFromClimb", mirrored),
        Robot.CURRENT_ROBOT_MODE == RobotMode.REAL
            ? new CMD_Shoot(
                    robot.conveyor,
                    robot.hood,
                    robot.intake,
                    robot.kicker,
                    robot.shooter,
                    0.2,
                    Math.toRadians(18000))
                .withTimeout(5)
            : new ShootFuelSim(robot.driveSimulation, robot.hood, robot.shooter),
        followPath("IntakeDepotFromOutpost", mirrored),
        Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? new CMD_Intake(robot.intake) : Commands.none(),
        followPath("ShootFromDepot", mirrored),
        Robot.CURRENT_ROBOT_MODE == RobotMode.REAL
            ? new CMD_Shoot(
                    robot.conveyor,
                    robot.hood,
                    robot.intake,
                    robot.kicker,
                    robot.shooter,
                    0.35,
                    Math.toRadians(21000))
                .withTimeout(5)
            : new ShootFuelSim(robot.driveSimulation, robot.hood, robot.shooter));
  }
}
