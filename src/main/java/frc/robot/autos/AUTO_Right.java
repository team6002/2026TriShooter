package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotMode;

public class AUTO_Right implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            Commands.runOnce(()-> robot.drive.setPose(getStartingPoseAtBlueAlliance()))
            ,followPath("getmiddleR1")
            ,followPath("gotolineR1")
            ,followPath("gotoHPR1")
            ,robot.drive.alignToTarget(()-> FieldConstants.getHubPose())
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                new ShootFuel(robot.drive, robot.conveyor, robot.intake, null, null, null) : 
                new ShootFuelSim(robot.driveSimulation)
            ,followPath("climbshootR1")
        );
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("getmiddleR1");
            return path.getStartingHolonomicPose().orElse(new Pose2d());
        }catch(Exception e){
            e.printStackTrace();
        }
        return new Pose2d();
    }
}
