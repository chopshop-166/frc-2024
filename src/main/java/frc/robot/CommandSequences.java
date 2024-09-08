package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.maps.subsystems.ArmRotateMap.ArmPresets;
import frc.robot.subsystems.ArmRotate;
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
        this.undertaker = undertaker;
    }

    // make sequences for intake and shooter.

    public Command intake() {
        return led.intakeSpinning().andThen(intake.intakeGamePiece(), led.grabbedPiece());
    }

    public Command shooterSpeed(Speeds speed) {
        return led.shooterSpinning().andThen(shooter.setSpeed(speed), led.shooterAtSpeed());
    }

    public Command armRotatePreset(ArmPresets presets) {
        return armRotate.moveTo(presets).alongWith(
                led.toPreset()).andThen(led.atPreset());
    }

    public Command moveAndIntake() {
        return this.armRotatePreset(ArmPresets.INTAKE)
                .andThen(this.intake().deadlineWith(undertaker.spinIn())).andThen(led.grabbedPiece());
    }

    public Command moveAndIntakeContingency() {
        return led.toPreset().andThen(this.intake().alongWith(undertaker.spinIn())).withTimeout(0.2);
    }

    public Command moveAndIntakeContingencyTwo() {
        return led.toPreset().andThen(this.intake().deadlineWith(undertaker.spinIn()).withTimeout(1))
                .andThen(this.armRotatePreset(ArmPresets.SCORE_SPEAKER_SUBWOOFER));
    }

    public Command autoGamePieceDetected() {
        return intake.getData().gamePieceDetected ? this.intake().deadlineWith(undertaker.spinIn())
                : armRotate.moveTo(ArmPresets.SCORE_SPEAKER_SUBWOOFER);
    }

    public Command feedShoot() {
        return shooterSpeed(Speeds.SUBWOOFER_SHOT).andThen(waitSeconds(.2),
                intake.feedShooter(),
                shooter.setSpeed(Speeds.OFF));
    }

    public Command outtake() {
        return intake.spinOut().alongWith(
                undertaker.spinOut(), led.outtake());

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

    public Command releaseAmp() {
        return intake.feedShooter().andThen(shooter.setSpeed(Speeds.OFF), this.armRotatePreset(ArmPresets.INTAKE),
                led.colorAlliance());
    }

    public Command scoreSpeakerAuto() {
        return this.shooterSpeed(Speeds.SUBWOOFER_SHOT).alongWith(
                this.armRotatePreset(ArmPresets.SCORE_SPEAKER_SUBWOOFER)).andThen(
                        intake.feedShooter(),
                        led.colorAlliance());
    }

    public Command rotateToIntake() {
        return this.armRotatePreset(ArmPresets.INTAKE);
    }

    public Command scoreSpeakerPodiumAuto() {
        return this.shooterSpeed(Speeds.PODIUM_SHOT).alongWith(
                this.armRotatePreset(ArmPresets.SCORE_SPEAKER_PODIUM)).andThen(
                        intake.feedShooter(),
                        led.colorAlliance());
    }

    public Command charge(Speeds speed, ArmPresets angle) {
        return this.shooterSpeed(speed).alongWith(armRotatePreset(angle));
    }

    public Command release() {
        return this.intake.feedShooter().andThen(this.shooterSpeed(Speeds.OFF), armRotatePreset(ArmPresets.INTAKE));
    }

    public Command setRumble(ButtonXboxController controller, int rumbleAmount) {
        return runOnce(() -> {
            controller.getHID().setRumble(RumbleType.kBothRumble, rumbleAmount);
        });
    }

    public Command autoShoot() {
        return this.drive.driveTillReadyToShoot()
                .alongWith(this.charge(Speeds.PODIUM_SHOT, ArmPresets.SCORE_SPEAKER_PODIUM))
                .andThen(this.release(), armRotatePreset(ArmPresets.INTAKE));

    }
}
