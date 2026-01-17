package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {

    private final DCMotorSim hoodSim;

    private final PIDController hoodPIDController =
            new PIDController(HoodConstants.kPSim, HoodConstants.kISim, HoodConstants.kDSim);
    private double reference = 0;
    public static double objectsInHopper = 0;

    public HoodIOSim() {
        hoodSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), .178, HoodConstants.kGearRatio),
                DCMotor.getNeo550(1));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodCurrent = getCurrent();
        inputs.hoodVoltage = getVoltage();
        inputs.hoodReference = getReference();
        inputs.hoodVelocity = Units.radiansToDegrees(getVelocity());
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
        hoodSim.setInputVoltage(voltage);
    }

    @Override
    public double getVoltage() {
        return hoodSim.getInputVoltage();
    }

    @Override
    public double getCurrent() {
        return hoodSim.getCurrentDrawAmps();
    }

    @Override
    public double getVelocity(){
        return hoodSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void PID() {
        hoodSim.setInput(hoodPIDController.calculate(hoodSim.getAngularVelocityRadPerSec(), reference)); 
    }

    @Override
    public void periodic() {
        hoodSim.update(0.02);
    }
}
