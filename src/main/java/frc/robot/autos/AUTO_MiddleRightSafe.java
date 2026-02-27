package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootCommand;
// import frc.robot.commands.drive.AutoAlignToMiddle;
import frc.robot.subsystems.intake.IntakeIOSim;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_MiddleRightSafe implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot, boolean mirrored)
            throws IOException, ParseException {
        return Commands.sequence(
                setAutoStartPose("gotoHPMRS", false, robot.drive),
                followPath("gotoHPMRS", false),
                new InstantCommand(() -> IntakeIOSim.putFuelInHopperSim(24)),
                new ShootCommand(robot)
                // ,new AutoAlignToMiddle(robot.drive)
                );
    }
}
