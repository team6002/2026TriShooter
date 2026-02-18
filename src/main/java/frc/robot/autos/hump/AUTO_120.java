package frc.robot.autos.hump;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.commands.ShootFuelSim;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

public class AUTO_120 implements Auto{
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("SwipeHalfMiddle", false, robot.drive)
            ,followPath("SwipeHalfMiddle", false)
            ,followPath("Shoot120", false)
            ,Robot.isReal() ?
                robot.superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT_120).andThen(new WaitCommand(5)) :
                new ShootFuelSim(robot.driveSimulation)
            ,followPath("SwipeMiddle", false)
            ,followPath("Shoot120Bottom", false)
            ,Robot.isReal() ?
                robot.superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT_120).andThen(new WaitCommand(5)) :
                new ShootFuelSim(robot.driveSimulation)
        );
    }
}