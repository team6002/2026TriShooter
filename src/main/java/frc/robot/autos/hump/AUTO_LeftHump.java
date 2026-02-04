package frc.robot.autos.hump;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.ShootFuelSim;
import frc.robot.commands.drive.AutoAlignToClimb;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotMode;

public class AUTO_LeftHump implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("gotomiddleSH1", true, robot.drive)
            ,followPath("gotomiddleSH1", true)
            ,followPath("grabmiddleSH1", true)
            ,robot.drive.alignToTarget(()-> FieldConstants.getHubPose())
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.SIM ? 
                new ShootFuelSim(robot.driveSimulation) :
                new ShootFuel(robot.drive, robot.conveyor, robot.intake, null, null, null)
            ,new AutoAlignToClimb(robot.drive)
        );
    }
}
