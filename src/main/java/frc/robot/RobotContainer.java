// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.autos.*;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.conveyor.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.IO.*;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.kicker.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.utils.AlertsManager;
import frc.robot.utils.CustomPIDs.MapleJoystickDriveInput;
import frc.robot.utils.constants.FieldConstants;
import frc.robot.utils.constants.RobotMode;

import java.util.function.IntSupplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final Shooter shooter;
    public final Intake intake;
    public final Conveyor conveyor;
    public final Kicker kicker;
    public final Hood hood;
    public final Vision vision;
    public final LEDStatusLight ledStatusLight;

    // public final AprilTagVision aprilTagVision;
    public SwerveDriveSimulation driveSimulation = null;

    private final Field2d field = new Field2d();
    // Controller
    public final DriverMap driver = new DriverMap.LeftHandedXbox(0);

    public Pose2d resetPose;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                    new GyroIONavX(),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3),
                    (pose) -> {});

                shooter = new Shooter(new ShooterIOSpark());
                intake = new Intake(new IntakeIOSpark());
                conveyor = new Conveyor(new ConveyorIOSpark());
                kicker = new Kicker(new KickerIOSpark());
                hood = new Hood(new HoodIOSpark());

                this.vision = new Vision(
                    drive,
                    new VisionIOPhotonVision(Vision_Constants.camera0Name, Vision_Constants.robotToCamera0)
                    ,new VisionIOPhotonVision(Vision_Constants.camera1Name, Vision_Constants.robotToCamera1)
                );

                break;
            case SIM:
                // create a maple-sim swerve drive simulation instance
                this.driveSimulation =
                    new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3.5, 4, new Rotation2d()));
                // add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                    new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIOSim(driveSimulation.getModules()[0]),
                    new ModuleIOSim(driveSimulation.getModules()[1]),
                    new ModuleIOSim(driveSimulation.getModules()[2]),
                    new ModuleIOSim(driveSimulation.getModules()[3]),
                    driveSimulation::setSimulationWorldPose);

                shooter = new Shooter(new ShooterIOSim());
                intake = new Intake(new IntakeIOSim(driveSimulation));
                conveyor = new Conveyor(new ConveyorIOSim());
                kicker = new Kicker(new KickerIOSim());
                hood = new Hood(new HoodIOSim());

                vision = new Vision(
                    drive
                    ,new VisionIOPhotonVisionSim(
                        Vision_Constants.camera0Name,
                        Vision_Constants.robotToCamera0,
                        driveSimulation::getSimulatedDriveTrainPose)
                    ,new VisionIOPhotonVisionSim(
                        Vision_Constants.camera1Name,
                        Vision_Constants.robotToCamera1,
                        driveSimulation::getSimulatedDriveTrainPose)
                );

                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    (pose) -> {});

                shooter = new Shooter(new ShooterIO() {});
                intake = new Intake(new IntakeIO() {});
                conveyor = new Conveyor(new ConveyorIO() {});
                kicker = new Kicker(new KickerIO() {});
                hood = new Hood(new HoodIO() {});

                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                break;
        }

        this.ledStatusLight = new LEDStatusLight(0, 155, true, false);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        try {
            autoChooser.addOption("Left Trench (Start intake facing bump, swipe our half, shoot, swipe other half, and shoot on opposite side)", new AUTO_Trench().getAutoCommand(this, false));
            autoChooser.addOption("Right Trench (Start intake facing bump, swipe our half, shoot, swipe other half, and shoot on opposite side)", new AUTO_Trench().getAutoCommand(this, true));

            autoChooser.addOption("Left Trench Shoot First (start shooter facing hub, shoot, swipe our half, come back through trench, shoot, swipe other half, and shoot on opposite side)", new AUTO_TrenchShootFirst().getAutoCommand(this, false));
            autoChooser.addOption("Right Trench Shoot First (start shooter facing hub, shoot, swipe our half, come back through trench, shoot, swipe other half, and shoot on opposite side)", new AUTO_TrenchShootFirst().getAutoCommand(this, true));

            autoChooser.addOption("Left Trench Opposing (Start intake facing bump, swipe our half, shoot, swipe opposing alliance's half, and shoot on same side)", new AUTO_TrenchOpposing().getAutoCommand(this, false));
            autoChooser.addOption("Right Trench Opposing (Start intake facing bump, swipe our half, shoot, swipe opposing alliance's half, and shoot on same side)", new AUTO_TrenchOpposing().getAutoCommand(this, true));

            autoChooser.addOption("Left Trench Bump (Start intake facing bump, swipe our half, back over bump, shoot, swipe other half, over bump on other half, and shoot on opposite side)", new AUTO_TrenchBump().getAutoCommand(this, false));
            autoChooser.addOption("Right Trench Bump (Start intake facing bump, swipe our half, back over bump, shoot, swipe other half, over bump on other half, and shoot on opposite side)", new AUTO_TrenchBump().getAutoCommand(this, true));

            autoChooser.addOption("Left Trench Opposing Bump (Start intake facing bump, swipe our half, come back over the bump, shoot, go back through trench, swipe other alliance's half, back over bump, shoot)", new AUTO_TrenchBumpOpposing().getAutoCommand(this, false));
            autoChooser.addOption("Right Trench Opposing Bump (Start intake facing bump, swipe our half, come back over the bump, shoot, go back through trench, swipe other alliance's half, back over bump, shoot)", new AUTO_TrenchBumpOpposing().getAutoCommand(this, true));

            autoChooser.addOption("Trench + Depot (Start intake facing bump on depot side, swipe middle, drive in front of depot, shoot, back into depot, and shoot again)", new AUTO_TrenchDepot().getAutoCommand(this, false));

            autoChooser.addOption("Outpost + Depot (Start intake facing driver station wall in the middle of right trench, run to outpost, wait for balls to drop, shoot, run over to depot, shoot)", new AUTO_OutpostAndDepot().getAutoCommand(this, false));

            autoChooser.addOption("Bump Left", new AUTO_Bump().getAutoCommand(this, false));
            autoChooser.addOption("Bump Right", new AUTO_Bump().getAutoCommand(this, true));
        } catch (Exception e) {
            AlertsManager.create("Auto Chooser failed to load: " + e.getMessage(), AlertType.kError);
            e.printStackTrace();
        }
        
        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();

        SmartDashboard.putData("Field", field);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    public void configureButtonBindings() {
        /* joystick drive command */
        final MapleJoystickDriveInput driveInput = driver.getDriveInput();
        IntSupplier pov = () -> -1;
        final JoystickDrive joystickDrive = new JoystickDrive(driveInput, () -> true, pov, drive);
        drive.setDefaultCommand(joystickDrive);

        //default commands
        shooter.setDefaultCommand(shooter.setTargetVelolcity(0));
        hood.setDefaultCommand(hood.setTargetPos(
            Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
            HoodConstants.kMinPos 
            : HoodConstants.kMinHoodAngle
        )); // sim uses radians, real uses rotations
        kicker.setDefaultCommand(kicker.runVoltage(KickerConstants.kOff));
        conveyor.setDefaultCommand(conveyor.runVoltage(ConveyorConstants.kOff));

        // Reset gyro / odometry
        final Runnable resetGyro = Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
            ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
                // reset odometry to actual robot pose during simulation
            : () -> drive.resetOdometry(
                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        driver.resetOdometryButton().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        driver.autoAlignmentButton().whileTrue(
            JoystickDriveAndAimAtTarget.driveAndAimAtTarget(
                driveInput
                ,drive
                ,()-> FieldConstants.getHubPose()
                ,ShooterConstants.kShooterOptimization
                ,0.5
                ,false
            )
        );

        driver.stopWithXButton().onTrue(Commands.runOnce(()-> drive.stopWithX()));

        if(Robot.CURRENT_ROBOT_MODE == RobotMode.REAL){
           driver.intakeButton().whileTrue(new CMD_Intake(intake));
           driver.yButton().onTrue(new CMD_Stow(intake));
           driver.aButton().onTrue(new CMD_Home(intake));

           driver.scoreButton().whileTrue(new CMD_Shoot(conveyor, hood, intake, kicker, shooter, 0.35, Math.toRadians(21000)));
           driver.rightBumper().whileTrue(new CMD_Shoot(conveyor, hood, intake, kicker, shooter, 0.2, Math.toRadians(18000)));
        }else if (Robot.CURRENT_ROBOT_MODE == RobotMode.SIM){
            driver.scoreButton().whileTrue(new ShootFuelSim(driveSimulation, hood, shooter));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Robot.CURRENT_ROBOT_MODE != RobotMode.SIM) return;

        if (FieldConstants.getAlliance() == Alliance.Blue) {
            resetPose = new Pose2d(3.5, 4, new Rotation2d());
        } else {
            resetPose = new Pose2d(13, 4, new Rotation2d());
        }

        drive.resetOdometry(resetPose);

        SimulatedArena.getInstance().resetFieldForAuto();
        IntakeIOSim.setFuelInHopper(8);
    }

    public void updateSimulation() {
        if (Robot.CURRENT_ROBOT_MODE != RobotMode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        Logger.recordOutput("FieldSimulation/Alliance", FieldConstants.getAlliance().toString());

        Logger.recordOutput("FieldSimulation/BlueScore", SimulatedArena.getInstance().getScore(true));
        Logger.recordOutput("FieldSimulation/RedScore", SimulatedArena.getInstance().getScore(false));
    }

    public static boolean motorBrakeEnabled = false;

    public void setMotorBrake(boolean brakeModeEnabled) {
        if (motorBrakeEnabled == brakeModeEnabled) return;

        System.out.println("Set motor brake: " + brakeModeEnabled);
        drive.setMotorBrake(brakeModeEnabled);

        motorBrakeEnabled = brakeModeEnabled;
    }

    public void updateTelemetryAndLED() {
        field.setRobotPose(
            Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
                ? driveSimulation.getSimulatedDriveTrainPose()
                : drive.getPose());
        if (Robot.CURRENT_ROBOT_MODE == RobotMode.SIM)
            field.getObject("Odometry").setPose(drive.getPose());

        AlertsManager.updateLEDAndLog(ledStatusLight);

        Logger.recordOutput("Hub Active", isHubActive());
        Logger.recordOutput("Match Time", DriverStation.getMatchTime());
    }

    public boolean isHubActive() {
        // 1. Hubs are always active during Auto, the 10s Transition, and the final 30s Endgame.
        if (DriverStation.isAutonomous()) return true;
        double time = DriverStation.getMatchTime();
        if (time > 130 || time <= 30) return true;

        // 2. Get FMS Data: Who goes inactive first?
        var myAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        boolean redInactiveFirst, weAreInactiveFirst;

        if(Robot.CURRENT_ROBOT_MODE == RobotMode.REAL){
            String gameData = DriverStation.getGameSpecificMessage();
            if (gameData.isEmpty()) return true; // Default to active if data hasn't arrived

            redInactiveFirst = gameData.charAt(0) == 'R';
            weAreInactiveFirst = (myAlliance == DriverStation.Alliance.Red) ? redInactiveFirst : !redInactiveFirst;
        }else{
            weAreInactiveFirst = true;
        }
        
        // 4. Check the 25-second shifts
        if (time > 105) return !weAreInactiveFirst; // Shift 1
        if (time > 80)  return weAreInactiveFirst;  // Shift 2
        if (time > 55)  return !weAreInactiveFirst; // Shift 3
        return weAreInactiveFirst;                  // Shift 4
    }
}
