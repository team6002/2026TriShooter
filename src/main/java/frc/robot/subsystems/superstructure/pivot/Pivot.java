package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AlertsManager;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
    // Hardware interface
    private final PivotIO io;
    private String name;
    private final PivotIOInputsAutoLogged inputs;

    // Alerts
    private final Alert pivotPrimaryFaultsAlert, pivotFollowerFaultsAlert;
    private final Alert configurationFailed, pivotAbsoluteEncoderDisconnectedAlert;

    public Pivot(PivotIO io, String name) {
        inputs = new PivotIOInputsAutoLogged();
        this.io = io;
        this.name = name;

        this.configurationFailed = AlertsManager.create(
                "Module-" + this.name + " configuration failed. Reboot robot after fixing connection.",
                Alert.AlertType.kError);
        this.pivotPrimaryFaultsAlert = AlertsManager.create(
                "Pivot '" + this.name + "' primary motor faults detected!", Alert.AlertType.kError);
        this.pivotFollowerFaultsAlert = AlertsManager.create(
                "Pivot '" + this.name + "' follower motor faults detected!", Alert.AlertType.kError);
        this.pivotAbsoluteEncoderDisconnectedAlert = AlertsManager.create(
                "Pivot '" + this.name + "' absolute encoder disconnected. (Check it post-match).",
                Alert.AlertType.kWarning);

        this.pivotPrimaryFaultsAlert.set(false);
        this.pivotFollowerFaultsAlert.set(false);
        this.pivotAbsoluteEncoderDisconnectedAlert.set(false);

        io.setMotorBrake(true);
        io.setIdle();
    }

    @Override
    public void periodic() {
        // Update inputs from IO and AdvantageKit.
        io.updateInputs(inputs);
        Logger.processInputs("Pivot/" + name, inputs);

        Logger.recordOutput("Pivot/" + name + "/Setpoint (Degrees)", inputs.setPoint.in(Degrees));
        Logger.recordOutput("Pivot/" + name + "/Current Position(Degrees)", inputs.currentAngle.in(Degrees));
        Logger.recordOutput("Pivot/" + name + "/Velocity (Degrees Per Second)", Math.toDegrees(getVelocityRadPerSec()));
        Logger.recordOutput("Pivot/" + name + "/At Reference", atReference());

        // Update alerts
        configurationFailed.set(inputs.configurationFailed);
        pivotPrimaryFaultsAlert.set(!inputs.primaryMotorConnected);
        pivotFollowerFaultsAlert.set(inputs.hasFollowerMotor && !inputs.followerMotorConnected);
        pivotAbsoluteEncoderDisconnectedAlert.set(!inputs.absoluteEncoderConnected);
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

    public boolean atReference(Angle setpoint) {
        double errorRad = inputs.currentAngle.minus(setpoint).in(Radians);
        return Math.abs(errorRad) < (0.1);
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

    public boolean trulyAtReference(Angle setpoint) {
        double errorRad = getArmAngle().minus(setpoint).in(Radians);
        return Math.abs(errorRad) < .01;
    }

    /** Request the arm to move to a given setpoint. */
    public void setTargetPosition(Angle setpoint) {
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
    public Command moveToPosition(Angle setpoint) {
        return run(() -> setTargetPosition(setpoint)).until(this::atReference);
    }

    /** @return the measured arm angle, where zero is horizontally forward. */
    public Angle getArmAngle() {
        return inputs.currentAngle;
    }

    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }

    /** Sets the brake mode of the arm motor. */
    public void setMotorBrake(boolean brakeModeEnabled) {
        io.setMotorBrake(brakeModeEnabled);
    }
}
