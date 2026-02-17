package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.RobotMode;

public class AUTO_Side implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            followPath("gotomiddleS1", false)
            ,followPath("grabfuelS1", false)
            ,followPath("gotolineS1", false)
            ,followPath("climbshootR1", false)
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                new ShootFuel(robot.drive, robot.conveyor, robot.intake, null, null, null) : 
                new ShootFuelSim(robot.driveSimulation)
        );
    }
}
