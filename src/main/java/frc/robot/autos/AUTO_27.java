package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.CMD_ShootFuelSim;

public class AUTO_27 implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("sweepmiddle27", false, robot.drive)
            ,followPath("sweepmiddle27", false)
            ,new CMD_ShootFuelSim(robot.driveSimulation, robot.intake)
            ,followPath("shootandsweepagain27", false)
            ,new CMD_ShootFuelSim(robot.driveSimulation, robot.intake)
        );
    }
}
