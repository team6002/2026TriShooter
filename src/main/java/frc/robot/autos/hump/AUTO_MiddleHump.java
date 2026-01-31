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
import frc.robot.constants.RobotMode;

public class AUTO_MiddleHump implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            Commands.runOnce(()-> robot.drive.setPose(getStartingPoseAtBlueAlliance()))
            ,followPath("gotomiddleM3")
            ,followPath("pickmiddleM3")
            ,followPath("gotolineM3")
            ,followPath("gotoHPM3")
            //auto align
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                new ShootFuel(robot.drive, robot.conveyor, robot.intake, null, null, null) : 
                new ShootFuelSim(robot.driveSimulation)
            ,followPath("climbshootM3")
        );
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("gotomiddleM3");
            return path.getStartingHolonomicPose().orElse(new Pose2d());
        }catch(Exception e){
            e.printStackTrace();
        }
        return new Pose2d();
    }
}
