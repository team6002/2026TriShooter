package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine sysIdRoutine;

    private final ShooterController leftController, middleController, rightController;

    private final LoggedNetworkNumber voltageRamp = new LoggedNetworkNumber(this.getName() + "/voltageRamp", ShooterConstants.kShootingVoltRamp);

    public Shooter(ShooterIO io) {
        this.io = io;
        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, 
                (state) -> Logger.recordOutput("/Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.baseUnitMagnitude()),
                null,
                this
            )
        );

        //initialize custom voltage controller
        leftController = new ShooterController(
            ShooterConstants.kLeftShooterS, 
            ShooterConstants.kLeftShooterV, 
            ShooterConstants.kShootingVoltRamp
        );
        middleController = new ShooterController(
            ShooterConstants.kMiddleShooterS, 
            ShooterConstants.kMiddleShootV, 
            ShooterConstants.kShootingVoltRamp
        );
        rightController = new ShooterController(
            ShooterConstants.kRightShooterS, 
            ShooterConstants.kRightShooterV,
            ShooterConstants.kShootingVoltRamp
        );

        stop();
    }

    public SysIdRoutine getSysIdRoutine() {
        return sysIdRoutine;
    }

    public double getReference() {
        return io.getReference();
    }

    public double getVelocity() {
        return io.getVelocity();
    }

    public double getCurrent() {
        return io.getCurrent();
    }

    public double getVoltage() {
        return io.getVoltage();
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void setReference(double velocity) {
        leftController.setTargetRpm(velocity);
        middleController.setTargetRpm(velocity);
        rightController.setTargetRpm(velocity);
    }

    public Command setTargetVelolcity(double velocity){
        return Commands.runOnce(()-> setReference(velocity), this);
    }

    public boolean isReady() {
       return middleController.isReady() && leftController.isReady() && rightController.isReady();
    }

    public void startSpinUp(){
        leftController.setSpinUp();
        middleController.setSpinUp();
        rightController.setSpinUp();
    }

    public void startShooting(){
        leftController.startShooting();
        middleController.startShooting();
        rightController.startShooting();
    }

    public void stop() {
       leftController.stop();
       middleController.stop();
       rightController.stop();

       io.setVoltage(0);
       io.setLeftVoltage(0);
       io.setRightVoltage(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        leftController.setVoltageRamp(voltageRamp.get());
        middleController.setVoltageRamp(voltageRamp.get());
        rightController.setVoltageRamp(voltageRamp.get());

        leftController.update(io.getLeftVelocity());
        middleController.update(io.getVelocity());
        rightController.update(io.getRightVelocity());

        io.setLeftVoltage(leftController.getVoltageCommand());
        io.setVoltage(middleController.getVoltageCommand());
        io.setRightVoltage(rightController.getVoltageCommand());

        io.periodic();

        Logger.processInputs(this.getName(), inputs);

        Logger.recordOutput(this.getName() + "/LeftVoltageCommand", leftController.getVoltageCommand());
        Logger.recordOutput(this.getName() + "/MiddleVoltageCommand", middleController.getVoltageCommand());
        Logger.recordOutput(this.getName() + "/RightVoltageCommand", rightController.getVoltageCommand());

        Logger.recordOutput(this.getName() + "/LeftControlStage", leftController.getControlStage().toString());
        Logger.recordOutput(this.getName() + "/MiddleControlStage", middleController.getControlStage().toString());
        Logger.recordOutput(this.getName() + "/RightControlStage", rightController.getControlStage().toString());

        Logger.recordOutput(this.getName() + "/LeftTargetVel", Units.radiansToDegrees(leftController.getTargetRpm()));
        Logger.recordOutput(this.getName() + "/MiddleTargetVel", Units.radiansToDegrees(middleController.getTargetRpm()));
        Logger.recordOutput(this.getName() + "/RightTargetVel", Units.radiansToDegrees(rightController.getTargetRpm()));
    }
}
