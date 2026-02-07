package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.ShooterIOSpark.ControlStage;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine sysIdRoutine;

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

    public void setFeedforwardSlot(double slot){
        io.setFeedforwardSlot(slot);
    }

    public void startShootingTimer(){
        io.startShootingTimer();
    }

    public void stopShootingTimer(){
        io.stopShootingTimer();
    }

    public void setControlStage(ControlStage controlStage){
        io.setControlStage(controlStage);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.PID();
        io.periodic();
        Logger.processInputs(this.getName(), inputs);
    }
}
