package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.Speeds;

public class CommandSequences {

    Drive drive;
    Intake intake;
    Shooter shooter;
    Led led;

    public CommandSequences(Drive drive, Intake intake, Shooter shooter, Led led) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.led = led;
    }

    // make sequences for intake and shooter.

    public Command intake() {
        return led.intakeSpinning().andThen(intake.intakeGamepiece(), led.grabbedPiece());
    }

    public Command shooter() {
        return led.shooterSpinning().andThen(shooter.setSpeed(Speeds.FULL_SPEED), intake.feedShooter(),
                led.colorAlliance());
    }

}
