package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.ArmRotate.ArmPresets;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Undertaker;
import frc.robot.subsystems.Shooter.Speeds;

public class CommandSequences {

    Drive drive;
    Intake intake;
    Shooter shooter;
    Led led;
    ArmRotate armRotate;
    Undertaker undertaker;

    public CommandSequences(Drive drive, Intake intake, Shooter shooter, Led led, ArmRotate armRotate,
            Undertaker undertaker) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.led = led;
        this.armRotate = armRotate;
    }

    // make sequences for intake and shooter.

    public Command intake() {
        return led.intakeSpinning().andThen(intake.intakeGamePiece(), led.grabbedPiece());
    }

    public Command shooterSpeed(Speeds speed) {
        return led.shooterSpinning().andThen(shooter.setSpeed(speed), led.shooterAtSpeed());
    }

    public Command armRotatePreset(ArmPresets presets) {
        return armRotate.moveTo(presets);
        // led.toPreset().andThen(
        // , led.atPreset());
    }

    public Command moveAndIntake() {
        return led.toPreset().andThen(this.intake().deadlineWith(
                armRotate.moveTo(ArmPresets.INTAKE).andThen(armRotate.moveToZero())), led.grabbedPiece());
    }

    public Command feedShoot() {
        return shooterSpeed(Speeds.SUBWOOFER_SHOT).andThen(waitSeconds(.5),
                intake.feedShooter(),
                shooter.setSpeed(Speeds.OFF));
    }

    public Command moveToIntake() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.INTAKE), led.atPreset());
    }

    public Command moveToAmp() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.SCORE_AMP), led.atPreset());
    }

    public Command stow() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.STOW), led.atPreset());
    }

    public Command moveToSpeaker() {
        return led.toPreset().andThen(armRotate.moveTo(ArmPresets.SCORE_SPEAKER_SUBWOOFER), led.atPreset());
    }

    public Command scoreAmp() {
        return this.armRotatePreset(ArmPresets.SCORE_AMP).withTimeout(0.75).andThen(this.shooterSpeed(Speeds.AMP_SPEED),
                intake.feedShooter(),
                shooter.setSpeed(Speeds.OFF), led.colorAlliance());
    }

    public Command scoreSpeakerCharge(ButtonXboxController controller2, ButtonXboxController controller1) {
        return this.shooterSpeed(Speeds.SUBWOOFER_SHOT).alongWith(
                this.armRotatePreset(ArmPresets.SCORE_SPEAKER_SUBWOOFER)).andThen(
                        setRumble(controller1, 1), setRumble(controller2, 1));
    }

    public Command scoreSpeakerRelease(ButtonXboxController controller2, ButtonXboxController controller1) {
        return intake.feedShooter().andThen(shooter.setSpeed(Speeds.OFF), setRumble(controller1, 0),
                setRumble(controller2, 0));
    }

    public Command scoreSpeakerAuto() {
        return this.shooterSpeed(Speeds.SUBWOOFER_SHOT).alongWith(
                this.armRotatePreset(ArmPresets.SCORE_SPEAKER_SUBWOOFER)).andThen(
                        intake.feedShooter(),
                        led.colorAlliance());
    }

    public Command podiumShot() {
        return this.shooterSpeed(Speeds.PODIUM_SHOT)
                .alongWith(this.armRotatePreset(ArmPresets.SCORE_SPEAKER_PODIUM))
                .andThen(intake.feedShooter(), shooterSpeed(Speeds.OFF));
    }

    public Command setRumble(ButtonXboxController controller, int rumbleAmount) {
        return runOnce(() -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, rumbleAmount);
        });
    }

    public Command test(ButtonXboxController controller) {
        return shooter.setSpeed(Speeds.HALF_SPEED).andThen(() -> {
            this.setRumble(controller, 1);
        });
    }

    public Command rumbleAndShoooterOff(ButtonXboxController controller) {
        return shooter.setSpeed(Speeds.HALF_SPEED).andThen(() -> {
            this.setRumble(controller, 0);
        });
    }
}
