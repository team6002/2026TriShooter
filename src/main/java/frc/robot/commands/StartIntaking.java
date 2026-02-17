package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;

public class StartIntaking extends Command{
    private final Conveyor conveyor;
    private final Intake intake;
    private final Kicker kicker;

    public StartIntaking(Conveyor conveyor, Intake intake, Kicker kicker) {
        this.conveyor = conveyor;
        this.intake = intake;
        this.kicker = kicker;
        addRequirements(conveyor, intake, kicker);
    }

    @Override
    public void initialize() {
        intake.setReference(IntakeConstants.kOn);
        conveyor.setReference(ConveyorConstants.kOff);
        kicker.setReference(KickerConstants.kOff);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
