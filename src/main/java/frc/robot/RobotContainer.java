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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.autos.*;
import frc.robot.commands.drive.*;
import frc.robot.constants.*;
import frc.robot.subsystems.conveyor.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.IO.*;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.kicker.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.utils.AlertsManager;
import frc.robot.utils.MapleJoystickDriveInput;

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
    public final SuperStructure superStructure;
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
                    // ,new VisionIOPhotonVisionSim(
                    //     Vision_Constants.camera0Name,
                    //     Vision_Constants.robotToCamera0,
                    //     driveSimulation::getSimulatedDriveTrainPose)
                    ,new VisionIOPhotonVisionSim(
                        Vision_Constants.camera1Name,
                        Vision_Constants.robotToCamera1,
                        driveSimulation::getSimulatedDriveTrainPose)
                    ,new VisionIOPhotonVisionSim(
                        Vision_Constants.camera2Name,
                        Vision_Constants.robotToCamera2,
                        driveSimulation::getSimulatedDriveTrainPose)
                    // ,new VisionIOPhotonVisionSim(
                    //     Vision_Constants.camera3Name,
                    //     Vision_Constants.robotToCamera3,
                    //     driveSimulation::getSimulatedDriveTrainPose)
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
        this.superStructure = new SuperStructure(conveyor, hood, intake, kicker, shooter);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        try {
            autoChooser.addOption("Trench Left", new AUTO_Trench().getAutoCommand(this, false));
            autoChooser.addOption("Trench Right", new AUTO_Trench().getAutoCommand(this, true));
            autoChooser.addOption("Bump Left", new AUTO_Bump().getAutoCommand(this, false));
            autoChooser.addOption("Bump Right", new AUTO_Bump().getAutoCommand(this, true));
        } catch (Exception e) {
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

        // Reset gyro / odometry
        final Runnable resetGyro = Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
            ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
                // reset odometry to actual robot pose during simulation
            : () -> drive.resetOdometry(
                // TODO: replace new Translation2d() with: drive.getPose().getTranslation()
                new Pose2d(new Translation2d(), new Rotation2d())); // zero gyro
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

        if(Robot.CURRENT_ROBOT_MODE == RobotMode.REAL){
            driver.scoreButton().whileTrue(superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT))
                .onFalse(superStructure.moveToPose(SuperStructurePose.EXTENDED));

            driver.rightBumper().whileTrue(superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT_120))
                .onFalse(superStructure.moveToPose(SuperStructurePose.EXTENDED));

            driver.intakeButton().onTrue(superStructure.moveToPose(SuperStructurePose.INTAKE))
                .onFalse(superStructure.moveToPose(SuperStructurePose.EXTENDED));

            driver.xButton().onTrue(superStructure.moveToPose(SuperStructurePose.HOME));

            driver.aButton().onTrue(superStructure.moveToPose(SuperStructurePose.STOW));
        }else if (Robot.CURRENT_ROBOT_MODE == RobotMode.SIM){
            driver.scoreButton().whileTrue(new ShootFuelSim(driveSimulation));
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
        // Logger.recordOutput("Vision/GetTarget", new VisionIOPhotonVisionSim(
        //                 Vision_Constants.camera0Name,
        //                 Vision_Constants.robotToCamera0,
        //                 driveSimulation::getSimulatedDriveTrainPose).);
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

        Logger.recordOutput("SuperStructure/goal", superStructure.targetPose().toString());
        Logger.recordOutput("SuperStructure/pose", superStructure.currentPose().toString());
    }
}
