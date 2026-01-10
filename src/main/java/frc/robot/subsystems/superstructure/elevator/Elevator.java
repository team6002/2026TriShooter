package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AlertsManager;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    // Hardware interface
    private final ElevatorIO io;
    private String name;
    private final ElevatorIOInputsAutoLogged inputs;

    // Alerts
    private final Alert armPrimaryFaultsAlert, armFollowerFaultsAlert;
    private final Alert armAbsoluteEncoderDisconnectedAlert;

    public Elevator(ElevatorIO io, String name) {
        inputs = new ElevatorIOInputsAutoLogged();
        this.io = io;
        this.name = name;

        this.armPrimaryFaultsAlert = AlertsManager.create(
                "Elevator '" + this.name + "' primary motor faults detected!", Alert.AlertType.kError);
        this.armFollowerFaultsAlert = AlertsManager.create(
                "Elevator '" + this.name + "' follower motor faults detected!", Alert.AlertType.kError);
        this.armAbsoluteEncoderDisconnectedAlert = AlertsManager.create(
                "Elevator '" + this.name + "' absolute encoder disconnected. (Check it post-match).",
                Alert.AlertType.kWarning);

        this.armPrimaryFaultsAlert.set(false);
        this.armFollowerFaultsAlert.set(false);
        this.armAbsoluteEncoderDisconnectedAlert.set(false);

        io.setMotorBrake(true);
    }

    @Override
    public void periodic() {
        // Update inputs from IO and AdvantageKit.
        io.updateInputs(inputs);
        Logger.processInputs("Elevator/" + name, inputs);

        Logger.recordOutput("Elevator/" + name + "/Setpoint (Inches)", inputs.setPoint.in(Inches));
        Logger.recordOutput("Elevator/" + name + "/Current Position(Inches)", inputs.currentHeight.in(Inches));
        Logger.recordOutput("Elevator/" + name + "/Velocity (Inches Per Second)", inputs.velocity.in(InchesPerSecond));
        Logger.recordOutput("Elevator/" + name + "/At Reference", atReference());

        // Update alerts
        armPrimaryFaultsAlert.set(!inputs.primaryMotorConnected);
        armFollowerFaultsAlert.set(inputs.hasFollowerMotor && !inputs.followerMotorConnected);
        // armAbsoluteEncoderDisconnectedAlert.set(!inputs.absoluteEncoderConnected);
    }

    public void runAutomatic() {
        io.runAutomatic();
    }

    public void runManual(double _power) {
        io.runManual(_power);
    }
    ;

    /**
     * Whether the arm's profile state is close enough to its setpoint.
     *
     * <p>Note that this does not reflect whether the mechanism is actually at its setpoint.
     *
     * @return <code>true</code> if there is a setpoint and the profile is close enough to the goal, <code>false</code>
     *     otherwise.
     */
    public boolean atReference() {
        return atReference(inputs.setPoint);
    }

    public boolean atReference(Distance setpoint) {
        double errorInches = inputs.currentHeight.minus(setpoint).in(Inches);
        return Math.abs(errorInches) < (2);
    }

    /**
     * Whether the mechanism is actually close enough to its setpoint.
     *
     * @return <code>true</code> if there is a setpoint and the measured arm angle is close enough to the goal, <code>
     *     false</code> otherwise.
     */
    public boolean trulyAtReference() {
        return trulyAtReference(inputs.setPoint);
    }

    public boolean trulyAtReference(Distance setpoint) {
        double errorInches = inputs.currentHeight.minus(setpoint).in(Inches);
        return Math.abs(errorInches) < .5;
    }

    /** Request the arm to move to a given setpoint. */
    public void setTargetPosition(Distance setpoint) {
        io.setTargetPosition(setpoint);
    }

    /**
     * Moves to a given angle, cancels the setpoint when reached.
     *
     * <p>When brake mode is on, the arm will still hold at that angle.
     *
     * <p><b>Note: This command finishes automatically when the setpoint is reached, causing the default command to
     * schedule.</b>
     */
    public Command moveToPosition(Distance setpoint) {
        return run(() -> setTargetPosition(setpoint)).until(this::atReference);
    }

    /** @return the measured arm angle, where zero is horizontally forward. */
    public Distance getArmAngle() {
        return inputs.currentHeight;
    }

    public LinearVelocity getVelocity() {
        return inputs.velocity;
    }

    /** Sets the brake mode of the arm motor. */
    public void setMotorBrake(boolean brakeModeEnabled) {
        io.setMotorBrake(brakeModeEnabled);
    }
}
