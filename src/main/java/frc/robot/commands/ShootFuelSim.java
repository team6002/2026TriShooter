package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.ShootingParams;
import frc.robot.constants.FieldConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ShootFuelSim extends Command {
    private final AbstractDriveTrainSimulation driveSim;
    private int timer;

    private static final Translation2d CENTER_SHOOTER_OFFSET = new Translation2d(Units.inchesToMeters(5.5), 0);
    private static final Translation2d LEFT_SHOOTER_OFFSET = new Translation2d(Units.inchesToMeters(5.5), Units.inchesToMeters(6));
    private static final Translation2d RIGHT_SHOOTER_OFFSET = new Translation2d(Units.inchesToMeters(5.5), Units.inchesToMeters(-6));
    private static final Translation2d[] SHOOTER_OFFSETS = {
        CENTER_SHOOTER_OFFSET,
        LEFT_SHOOTER_OFFSET,
        RIGHT_SHOOTER_OFFSET
    };

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
        if (timer >= 5 && IntakeIOSim.numObjectsInHopper() > 0) {
            Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();
            
            int maxShots = Math.min(3, IntakeIOSim.numObjectsInHopper());
            int numShots = 1 + (int)(Math.random() * maxShots);
            
            List<Integer> laneIndices = new ArrayList<>();
            for (int i = 0; i < 3; i++) {
                laneIndices.add(i);
            }
            Collections.shuffle(laneIndices);
            
            for (int i = 0; i < numShots; i++) {
                Translation2d shooterOffset = SHOOTER_OFFSETS[laneIndices.get(i)];
                
                ShootingParams params = calculateLeadingParams(robotPose, shooterOffset, FieldConstants.getHubPose());

                IntakeIOSim.obtainFuelFromHopper();

                SimulatedArena.getInstance().addGamePieceProjectile(
                    new RebuiltFuelOnFly(
                        robotPose.getTranslation(),
                        shooterOffset,
                        driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        robotPose.getRotation(),
                        Inches.of(21),
                        MetersPerSecond.of(params.velocityMPS()),
                        Radians.of(params.angRad())
                    )
                );
            }

            timer = 0;
        } else {
            timer++;
        }
    }

    @Override
    public boolean isFinished(){
        return IntakeIOSim.numObjectsInHopper() <= 0;
    }

    private ShootingParams calculateLeadingParams(Pose2d robotPose, Translation2d shooterOffset, Translation2d hubPosition) {
        Translation2d shooterOffsetField = shooterOffset.rotateBy(robotPose.getRotation());
        Translation2d shooterPos = robotPose.getTranslation().plus(shooterOffsetField);
        
        ChassisSpeeds speeds = driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        
        Translation2d relativeVel = robotVel.unaryMinus();

        double distance = shooterPos.getDistance(hubPosition);
        ShootingParams params = ShooterConstants.getShootingParams(distance);
        
        Translation2d targetPos = hubPosition;
        
        for (int i = 0; i < 10; i++) {
            double predictedDistance = shooterPos.getDistance(targetPos);
            params = ShooterConstants.getShootingParams(predictedDistance);
            
            double projSpeed = Math.cos(params.angRad()) * params.velocityMPS();
            
            Translation2d toTarget = targetPos.minus(shooterPos);
            double t = toTarget.getNorm() / projSpeed;
            targetPos = hubPosition.plus(relativeVel.times(t));

            Logger.recordOutput("Lead/currentDistance", distance);
            Logger.recordOutput("Lead/predictedDistance", predictedDistance);
            Logger.recordOutput("Lead/flightTime", params.tofSeconds());
            Logger.recordOutput("Lead/robotVelX", robotVel.getX());
            Logger.recordOutput("Lead/robotVelY", robotVel.getY());
        }

        return params;
    }
}