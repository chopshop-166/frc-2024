package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ArmRotate.ArmPresets;
import frc.robot.subsystems.Shooter.Speeds;

public class CommandSequences {

    Drive drive;
    Intake intake;
    Shooter shooter;
    Led led;
    ArmRotate armRotate;

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

    public Command moveToIntake() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.INTAKE), led.atPreset());
    }

    public Command moveToAmp() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.SCORE_AMP), led.atPreset());
    }

    public Command moveToSpeaker() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.SCORE_SPEAKER_SUBWOOFER), led.atPreset());
    }

}
