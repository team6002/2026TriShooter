package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ClimbIOSpark implements ClimbIO {
  private final SparkMax climbMotor;
  private final RelativeEncoder climbEncoder;
  private final SparkClosedLoopController climbController;

  private double climbReference;
  private ControlType climbType;

  public ClimbIOSpark() {
    // initialize motor
    climbMotor = new SparkMax(ClimbConstants.kClimbCanId, MotorType.kBrushless);

    // initialize PID controller
    climbController = climbMotor.getClosedLoopController();

    // initalize encoder
    climbEncoder = climbMotor.getEncoder();

    // apply config
    climbMotor.configure(
        ClimbConfig.climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // reset target speed in init
    climbReference = 0;
    climbType = ControlType.kPosition;
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.climbReference = getReference();
    inputs.climbCurrent = getCurrent();
    inputs.climbVoltage = getVoltage();
    inputs.climbVelocity = getVelocity();
    inputs.climbTemp = Fahrenheit.convertFrom(climbMotor.getMotorTemperature(), Celsius);
  }

  @Override
  public double getVelocity() {
    return climbEncoder.getVelocity();
  }

  @Override
  public double getCurrent() {
    return climbMotor.getOutputCurrent();
  }

  @Override
  public double getVoltage() {
    return climbMotor.getBusVoltage() * climbMotor.getAppliedOutput();
  }

  @Override
  public double getReference() {
    return climbReference;
  }

  @Override
  public void setVoltage(double voltage) {
    climbReference = voltage;
    climbType = ControlType.kVoltage;
  }

  @Override
  public void setReference(double velocity) {
    climbReference = velocity;
    climbType = ControlType.kVelocity;
  }

  @Override
  public void periodic() {
    climbController.setSetpoint(climbReference, climbType, ClosedLoopSlot.kSlot0);
  }
}
