package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_TrenchAndOutpost implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("SweepHalfMiddle", false, robot.drive),
        // intake and sweep half the middle
        new ParallelCommandGroup(
            new CMD_Intake(robot.intake), followPath("SweepHalfMiddle", false)),
        // turn off intake and run back to our side to shoot
        new CMD_Extend(robot.intake),
        followPath("ShootTrench", false),
        // shoot for 2 seconds and then run to outpost
        // robot.shootClose().withTimeout(4),
        followPath("IntakeOutpostFromTrenchShoot", false),
        new CMD_Extend(robot.intake),
        // wait a short time to let HP dump the outpost into our hopper
        new WaitCommand(1),
        // run back to the same shooting spot from earlier and unload until auto ends
        followPath("ShootFromOutpostTrench", false)
        // robot.shootClose()
        );
  }
}
