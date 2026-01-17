package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {

    private final DCMotorSim climbSim;

    private final PIDController climbPIDController =
            new PIDController(ClimbConstants.kPSim, ClimbConstants.kISim, ClimbConstants.kDSim);
    private final SimpleMotorFeedforward climbFeedforward =
            new SimpleMotorFeedforward(ClimbConstants.kS, ClimbConstants.kV,
                    ClimbConstants.kA);
    private double reference = 0;
    public static double objectsInHopper = 0;

    public ClimbIOSim() {
        climbSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .178, ClimbConstants.kGearRatio),
                DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.climbCurrent = getCurrent();
        inputs.climbVoltage = getVoltage();
        inputs.climbReference = getReference();
        inputs.climbVelocity = Units.radiansToDegrees(getVelocity());
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
        climbSim.setInputVoltage(voltage);
    }

    @Override
    public double getVoltage() {
        return climbSim.getInputVoltage();
    }

    @Override
    public double getCurrent() {
        return climbSim.getCurrentDrawAmps();
    }

    @Override
    public double getVelocity(){
        return climbSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void PID() {
        climbSim.setInput(
            climbPIDController.calculate(getVelocity(), reference) 
            + climbFeedforward.calculateWithVelocities(getVelocity(), reference)
        ); 
    }

    @Override
    public void periodic() {
        climbSim.update(0.02);
    }
}
