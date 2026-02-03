package frc.robot.autos.hump;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotMode;

public class AUTO_SideHump implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        Command cmds;
        cmds = Commands.sequence(
            setAutoStartPose("gotomiddleSH1", false, robot.drive)
            ,followPath("gotomiddleSH1")
            ,followPath("grabmiddleSH1")
            ,robot.drive.alignToTarget(()-> FieldConstants.getHubPose())
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.SIM ? 
                new ShootFuelSim(robot.driveSimulation) :
                new ShootFuel(robot.drive, robot.conveyor, robot.intake, null, null, null)
            ,followPath("climbSH1")
        );

        cmds.setName("test");
        return cmds;
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("gotomiddleSH1");
            return path.getStartingHolonomicPose().orElse(new Pose2d());
        }catch(Exception e){
            e.printStackTrace();
        }
        return new Pose2d();
    }
}
