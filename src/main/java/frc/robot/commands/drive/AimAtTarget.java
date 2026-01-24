package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.ChassisHeadingController;
import frc.robot.utils.ChassisHeadingController.FaceToTargetRequest;
import com.pathplanner.lib.util.DriveFeedforwards;

public class AimAtTarget extends Command {
    private final Drive drive;
    private final Translation2d targetPose;
    private final Timer settleTimer = new Timer();
    private static final double SETTLE_TIME = 0.15; // Time to stay at setpoint before finishing

    public AimAtTarget(Drive drive, Translation2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        ChassisHeadingController.getInstance()
            .setHeadingRequest(new FaceToTargetRequest(() -> targetPose, null));
        settleTimer.restart();
    }

    @Override
    public void execute() {
        drive.runRobotCentricSpeedsWithFeedforwards(
            new ChassisSpeeds(), 
            DriveFeedforwards.zeros(4)
        );
        
        // Reset timer if not at setpoint
        if (!ChassisHeadingController.getInstance().atSetPoint()) {
            settleTimer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        // Only finish after being at setpoint for SETTLE_TIME
        return ChassisHeadingController.getInstance().atSetPoint() 
            && settleTimer.hasElapsed(SETTLE_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        ChassisHeadingController.getInstance()
            .setHeadingRequest(new ChassisHeadingController.NullRequest());
        drive.stop();
    }
}