package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeIOSim implements IntakeIO {

  private final DCMotorSim intakeSim;
  private final PIDController intakePIDController =
      new PIDController(IntakeConstants.kPSim, IntakeConstants.kISim, IntakeConstants.kDSim);
  private final SimpleMotorFeedforward intakeFeedforward =
      new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);
  private static IntakeSimulation intakeSimulation;
  private double reference = 0;

  private final DCMotorSim intakeExtenderSim;
  private final PIDController intakeExtenderPIDController =
      new PIDController(ExtenderConstants.kPSim, ExtenderConstants.kISim, ExtenderConstants.kDSim);
  private final SimpleMotorFeedforward intakeExtenderFeedforward =
      new SimpleMotorFeedforward(ExtenderConstants.kS, ExtenderConstants.kV, IntakeConstants.kA);
  private double extenderReference = 0;

  public static double objectsInHopper = 0;

  private final LoggedMechanism2d intakeMechanism;
  private final LoggedMechanismRoot2d intakeRoot;
  private final LoggedMechanismLigament2d intakeVisualizer =
      new LoggedMechanismLigament2d(
          "intake", Inches.of(1), Degrees.of(-87.5), 50, new Color8Bit(255, 92, 0));

  public IntakeIOSim(AbstractDriveTrainSimulation driveSim) {
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .178, IntakeConstants.kGearRatio),
            DCMotor.getNEO(1));

    intakeExtenderSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .1, ExtenderConstants.kGearRatio),
            DCMotor.getNEO(1));

    intakeMechanism = new LoggedMechanism2d(Inches.of(24), Inches.of(10));
    intakeRoot =
        intakeMechanism.getRoot("Intake", Units.inchesToMeters(-16), Units.inchesToMeters(0));
    intakeRoot.append(intakeVisualizer);

    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel", driveSim, Inches.of(24), Inches.of(10), IntakeSide.BACK, 48);

    intakeSimulation.startIntake();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeCurrent = getCurrent();
    inputs.intakeVoltage = getVoltage();
    inputs.intakeReference = getReference();
    inputs.intakeVelocity = Units.radiansToDegrees(getVelocity());
    inputs.intakePosition = Units.radiansToDegrees(getPosition());

    inputs.extenderCurrent = getExtenderCurrent();
    inputs.extenderVoltage = getExtenderVoltage();
    inputs.extenderReference = getExtenderReference();
    inputs.extenderVelocity = Units.radiansToDegrees(getExtenderVelocity());
    inputs.extenderPosition = Units.radiansToDegrees(getExtenderPosition());
    inputs.extenderInPosition = getExtenderInPosition();
  }

  @Override
  public void setReference(double reference) {
    this.reference = reference;
  }

  @Override
  public double getReference() {
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
  public double getVelocity() {
    return intakeSim.getAngularVelocityRadPerSec();
  }

  @Override
  public double getPosition() {
    return intakeSim.getAngularPositionRad();
  }

  @Override
  public void setExtenderReference(double reference) {
    this.extenderReference = reference;
  }

  @Override
  public double getExtenderReference() {
    return extenderReference;
  }

  @Override
  public void setExtenderVoltage(double voltage) {
    intakeExtenderSim.setInputVoltage(voltage);
  }

  @Override
  public double getExtenderVoltage() {
    return intakeExtenderSim.getInputVoltage();
  }

  @Override
  public double getExtenderCurrent() {
    return intakeExtenderSim.getCurrentDrawAmps();
  }

  @Override
  public double getExtenderVelocity() {
    return intakeExtenderSim.getAngularVelocityRadPerSec();
  }

  @Override
  public double getExtenderPosition() {
    return intakeExtenderSim.getAngularPositionRad();
  }

  @Override
  public boolean getExtenderInPosition() {
    // Math.abs(getExtenderPosition() - getExtenderReference()) < ExtenderConstants.kTolerance;
    return true;
  }

  @Override
  public void periodic() {
    intakeSim.setInput(
        intakePIDController.calculate(intakeSim.getAngularVelocityRadPerSec(), reference)
            + intakeFeedforward.calculateWithVelocities(getVelocity(), reference));

    intakeExtenderSim.setInput(
        intakeExtenderPIDController.calculate(
                intakeExtenderSim.getAngularVelocityRadPerSec(), reference)
            + intakeExtenderFeedforward.calculateWithVelocities(getVelocity(), reference));

    intakeSim.update(0.02);
    intakeExtenderSim.update(0.02);

    Logger.recordOutput("Intake/FuelInHopper", numObjectsInHopper());
    Logger.recordOutput("IntakeVisualizer", intakeMechanism);
  }

  public static boolean obtainFuelFromHopper() {
    return intakeSimulation.obtainGamePieceFromIntake();
  }

  public static int numObjectsInHopper() {
    return intakeSimulation.getGamePiecesAmount();
  }

  public static void putFuelInHopperSim(int fuel) {
    setFuelInHopper(numObjectsInHopper() + fuel);
  }

  public static void setFuelInHopper(int fuel) {
    intakeSimulation.setGamePiecesCount(fuel);
  }
}
