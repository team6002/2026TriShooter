package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShootFuel extends Command{
    private final Conveyor conveyor;
    private final Intake intake;
    private final Kicker kicker;
    private final Hood hood;
    private final Shooter shooter;
    private final Drive drive;

    public ShootFuel(Drive drive, Conveyor conveyor, Intake intake, Kicker kicker, Hood hood, Shooter shooter) {
        this.drive = drive;
        this.conveyor = conveyor;
        this.intake = intake;
        this.kicker = kicker;
        this.hood = hood;
        this.shooter = shooter;
        addRequirements(conveyor, intake, kicker, hood, shooter);
    }

    @Override
    public void initialize() {
        intake.setReference(IntakeConstants.kOff);
        conveyor.setReference(ConveyorConstants.kConvey);
        kicker.setReference(KickerConstants.kKick);
    }

    @Override
    public void execute() {
        // calculate distance to hub
        double distance = drive.getPose().getTranslation().getDistance(FieldConstants.getHubPose());

        ShooterConstants.ShootingParams params = ShooterConstants.getShootingParams(distance);

        // set shooter and hood references based on distance
        shooter.setReference(params.velocityMPS());
        hood.setReference(params.angRad());
    }

    @Override
    public boolean isFinished() {
        // this command ends either when interrupted by another command or when shoot button is released
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVoltage(ShooterConstants.kHolding);
    }
}
