package frc.robot.autos.trench;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.commands.ShootCommand;
// import frc.robot.commands.drive.AutoAlignToMiddle;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_MiddleLeftSafe implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot, boolean mirrored) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("gotodepotMLS", false, robot.drive)
            ,followPath("gotodepotMLS", false)
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,new ShootCommand(robot)
            // ,new AutoAlignToMiddle(robot.drive)
        );
    }
}
