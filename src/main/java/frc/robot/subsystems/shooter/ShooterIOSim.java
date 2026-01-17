package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

    private final DCMotorSim shooterSim;
    private final DCMotorSim leftShooterSim;
    private final DCMotorSim rightShooterSIm;

    private final PIDController shooterPIDController =
            new PIDController(ShooterConstants.kPSim, ShooterConstants.kISim, ShooterConstants.kDSim);
    private final PIDController leftShooterPIDController =
            new PIDController(ShooterConstants.kPSim, ShooterConstants.kISim, ShooterConstants.kDSim);
    private final PIDController rightShooterPIDController =
            new PIDController(ShooterConstants.kPSim, ShooterConstants.kISim, ShooterConstants.kDSim);
    private double reference = 0;
    public static double objectsInHopper = 0;

    public ShooterIOSim() {
        shooterSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .178, ShooterConstants.kGearRatio),
                DCMotor.getNEO(1));

        leftShooterSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .178, ShooterConstants.kGearRatio),
                DCMotor.getNEO(1));

        rightShooterSIm = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .178, ShooterConstants.kGearRatio),
                DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterCurrent = getCurrent();
        inputs.shooterVoltage = getVoltage();
        inputs.shooterReference = getReference();
        inputs.shooterVelocity = Units.radiansToDegrees(getVelocity());
    }

    @Override
    public void setReference(double reference) {
        this.reference = reference;
    }

    @Override
    public double getReference(){
        return reference;
    }

    @Override
    public void setVoltage(double voltage) {
        shooterSim.setInputVoltage(voltage);
        leftShooterSim.setInputVoltage(voltage);
        rightShooterSIm.setInputVoltage(voltage);
    }

    @Override
    public double getVoltage() {
        return shooterSim.getInputVoltage();
    }

    @Override
    public double getCurrent() {
        return shooterSim.getCurrentDrawAmps();
    }

    @Override
    public double getVelocity(){
        return shooterSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void PID() {
        shooterSim.setInput(shooterPIDController.calculate(shooterSim.getAngularVelocityRadPerSec(), reference));
        leftShooterSim.setInput(leftShooterPIDController.calculate(leftShooterSim.getAngularVelocityRadPerSec(), reference));
        rightShooterSIm.setInput(rightShooterPIDController.calculate(rightShooterSIm.getAngularVelocityRadPerSec(), reference));
    }

    @Override
    public void periodic() {
        shooterSim.update(0.02);
        leftShooterSim.update(.02);
        rightShooterSIm.update(.02);
    }
}
