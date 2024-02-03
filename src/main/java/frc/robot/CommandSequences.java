package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.ArmRotate.ArmPresets;
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
    ArmRotate armRotate;

    public CommandSequences(Drive drive, Intake intake, Shooter shooter, Led led, ArmRotate armRotate) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.led = led;
        this.armRotate = armRotate;
    }

    // make sequences for intake and shooter.

    public Command intake() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.INTAKE), led.atPreset(), led.intakeSpinning(),
                intake.intakeGamepiece(), led.grabbedPiece());
    }

    public Command shooter() {
        return led.shooterSpinning().andThen(shooter.setSpeed(Speeds.FULL_SPEED), intake.feedShooter(),
                led.colorAlliance());
    }

    public Command feedShoot() {
        return shooter.setSpeed(Speeds.THREE_QUARTER_SPEED).andThen(waitSeconds(.5), intake.feedShooter(),
                shooter.setSpeed(Speeds.OFF));
    }

    public Command moveToIntake() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.INTAKE), led.atPreset());
    }

    public Command moveToAmp() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.SCORE_AMP), led.atPreset());
    }

    public Command scoreAmp() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.SCORE_AMP), led.atPreset(), led.shooterSpinning(),
                shooter.setSpeed(Speeds.SLOW_SPEED), intake.feedShooter(),
                shooter.setSpeed(Speeds.OFF), led.colorAlliance());
    }

    public Command moveToSpeaker() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.SCORE_SPEAKER_SUBWOOFER), led.atPreset());
    }

    public Command scoreSpeaker() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.SCORE_SPEAKER_SUBWOOFER), led.atPreset(),
                led.shooterSpinning(), shooter.setSpeed(Speeds.THREE_QUARTER_SPEED), intake.feedShooter(),
                shooter.setSpeed(Speeds.OFF), led.colorAlliance());
    }

    public Command stow() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.STOW), led.atPreset());
    }
}
