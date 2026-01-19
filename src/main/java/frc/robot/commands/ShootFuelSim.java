package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingParams;
import frc.robot.constants.FieldConstants;

public class ShootFuelSim extends Command {
    private final AbstractDriveTrainSimulation driveSim;
    private int timer;

    public ShootFuelSim(AbstractDriveTrainSimulation driveSim){
        this.driveSim = driveSim;
    }

    @Override
    public void initialize(){
        timer = 0;
        if(!RobotBase.isSimulation()) return;
    }

    @Override
    public void execute(){
        if (timer > 3 && IntakeIOSim.numObjectsInHopper() > 0) {

            Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();

            // Calculate predicted target position and use that distance for params
            ShootingResult result = calculateShot(robotPose, FieldConstants.HubPose);

            IntakeIOSim.obtainFuelFromHopper();

            SimulatedArena.getInstance().addGamePieceProjectile(
                new RebuiltFuelOnFly(
                    robotPose.getTranslation(),
                    new Translation2d(), // no offset for now
                    driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    robotPose.getRotation().plus(result.turretAngleRobot),
                    Inches.of(6),
                    MetersPerSecond.of(result.params.velocityMPS()),
                    Radians.of(result.params.angRad())
                )
            );

            timer = 0;
        } else {
            timer++;
        }
    }

    private record ShootingResult(Rotation2d turretAngleRobot, ShootingParams params) {}

    private ShootingResult calculateShot(Pose2d robotPose, Translation2d hubPosition) {
        Translation2d shooterPos = robotPose.getTranslation();
        
        // Robot velocity (field-relative)
        ChassisSpeeds speeds = driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        
        // Target moves opposite robot motion
        Translation2d relativeVel = robotVel.unaryMinus();

        // Iterative solution: refine both target position AND shooting params
        // Start with current distance to hub
        double distance = shooterPos.getDistance(hubPosition);
        ShootingParams params = ShooterConstants.getShootingParams(distance);
        
        Translation2d targetPos = hubPosition;
        
        // Iterate to converge on both target position and correct params
        for (int i = 0; i < 10; i++) {
            // Recalculate params based on predicted distance
            double predictedDistance = shooterPos.getDistance(targetPos);
            params = ShooterConstants.getShootingParams(predictedDistance);
            
            // Horizontal projectile speed with updated params
            double projSpeed = Math.cos(params.angRad()) * params.velocityMPS();
            
            // Update target position based on flight time
            Translation2d toTarget = targetPos.minus(shooterPos);
            double t = toTarget.getNorm() / projSpeed;
            targetPos = hubPosition.plus(relativeVel.times(t));
        }

        // Final angle calculation
        Translation2d finalVec = targetPos.minus(shooterPos);
        Rotation2d fieldAngle = finalVec.getAngle();
        Rotation2d turretAngleRobot = fieldAngle.minus(robotPose.getRotation());

        return new ShootingResult(turretAngleRobot, params);
    }
}