package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.CMD_ShootFuelSim;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_9470automirrored implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("swipemiddle9470", true, robot.drive),
        followPath("swipemiddle9470", true),
        followPath("shootfirst9470", true),
        new CMD_ShootFuelSim(robot.driveSimulation, robot.intake),
        followPath("secondsweep9470", true),
        followPath("shootsecond9470", true),
        new CMD_ShootFuelSim(robot.driveSimulation, robot.intake));
  }
}
