package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.ShootFuelSim;
import frc.robot.commands.drive.AutoAlignToClimb;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotMode;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_Right implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("getmiddleR1", false, robot.drive)
            ,followPath("getmiddleR1", false)
            ,followPath("gotolineR1", false)
            ,followPath("gotoshootR1", false)
            ,robot.drive.alignToTarget(()-> FieldConstants.getHubPose())
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                new ShootFuel(robot.drive, robot.conveyor, robot.intake, null, null, null) : 
                new ShootFuelSim(robot.driveSimulation)
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,followPath("gotoHPR1", false)
            ,robot.drive.alignToTarget(()-> FieldConstants.getHubPose())
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                new ShootFuel(robot.drive, robot.conveyor, robot.intake, null, null, null) : 
                new ShootFuelSim(robot.driveSimulation)
            ,new AutoAlignToClimb(robot.drive)
        );
    }
}
