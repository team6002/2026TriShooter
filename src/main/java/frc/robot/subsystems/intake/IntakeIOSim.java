package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

    private final DCMotorSim intakeSim;
    private final PIDController intakePIDController =
            new PIDController(IntakeConstants.kPSim, IntakeConstants.kISim, IntakeConstants.kDSim);
    private static IntakeSimulation intakeSimulation;
    private double reference = 0;
    public static double objectsInHopper = 0;

    public IntakeIOSim(AbstractDriveTrainSimulation driveSim) {
        intakeSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), .178, IntakeConstants.kGearRatio),
                DCMotor.getNeo550(2));

        intakeSimulation = IntakeSimulation.InTheFrameIntake("Fuel", driveSim,
            Inches.of(20), IntakeSide.FRONT, 48);

        intakeSimulation.startIntake();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeCurrent = getCurrent();
        inputs.intakeVoltage = getVoltage();
        inputs.intakeReference = getReference();
        inputs.intakeVelocity = Units.radiansToDegrees(getVelocity());
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
        intakeSim.setInputVoltage(voltage);
    }

    @Override
    public double getVoltage() {
        return intakeSim.getInputVoltage();
    }

    @Override
    public double getCurrent() {
        return intakeSim.getCurrentDrawAmps();
    }

    @Override
    public void PID() {
        intakeSim.setInput(intakePIDController.calculate(intakeSim.getAngularVelocityRadPerSec(), reference));
    }

    @Override
    public void periodic() {
        intakeSim.update(0.02);

        Logger.recordOutput("Intake/FuelInHopper",  numObjectsInHopper());
    }

    public static boolean obtainFuelFromHopper(){
        return intakeSimulation.obtainGamePieceFromIntake();
    }

    public static int numObjectsInHopper(){
        return intakeSimulation.getGamePiecesAmount();
    }

    public static void putFuelInHopperSim(int fuel) {
        for (int i = 0; i < fuel; i++) intakeSimulation.addGamePieceToIntake();
    }
}
