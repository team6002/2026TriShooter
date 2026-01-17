package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final SysIdRoutine sysIdRoutine;

    public Intake(IntakeIO io) {
        this.io = io;
        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, 
                (state) -> Logger.recordOutput("/Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.baseUnitMagnitude()),
                null,
                this
            )
        );
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
        io.setReference(velocity);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // io.PID();
        io.periodic();
        Logger.processInputs("Intake", inputs);
    }
}
