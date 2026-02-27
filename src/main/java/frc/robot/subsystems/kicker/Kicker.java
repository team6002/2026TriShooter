package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    private final SysIdRoutine sysIdRoutine;

    public Kicker(KickerIO io) {
        this.io = io;
        this.sysIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) ->
                                        Logger.recordOutput(
                                                "/Kicker/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> io.setVoltage(voltage.baseUnitMagnitude()),
                                null,
                                this));
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

    public Command runVoltage(double voltage) {
        return Commands.runOnce(() -> setVoltage(voltage), this);
    }

    public void setReference(double velocity) {
        io.setReference(velocity);
    }

    public Command setTargetVel(double vel) {
        return Commands.runOnce(() -> setReference(vel), this);
    }

    public boolean atVelocity() {
        return io.atVelocity();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.periodic();
        Logger.processInputs(this.getName(), inputs);
    }
}
