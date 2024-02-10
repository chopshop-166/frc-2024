package frc.robot.subsystems;

import com.chopshop166.chopshoplib.leds.LEDSubsystem;
import com.chopshop166.chopshoplib.leds.patterns.AlliancePattern;
import com.chopshop166.chopshoplib.leds.patterns.ColdFirePattern;
import com.chopshop166.chopshoplib.leds.patterns.FlashPattern;
import com.chopshop166.chopshoplib.leds.patterns.SpinPattern;
import com.chopshop166.chopshoplib.maps.LedMap;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds.NewFlashPattern;

public class Led extends LEDSubsystem {

    public Led(LedMap map) {
        super(map);
        // This one is length / 2 because the buffer has a mirrored other half
        ledBuffer.setPattern("Fire", new ColdFirePattern(ledBuffer.getLength() / 2));

    }

    public Command colorAlliance() {
        return setPattern("Alliance", new AlliancePattern(), "Alliance");
    }

    public Command coloralliance() {
        return setGlobalPattern(new AlliancePattern());
    }

    public Command resetColor() {
        return setGlobalPattern(new Color(201, 198, 204));
    }

    public Command intakeSpinning() {
        return setPattern("Intake", new SpinPattern(), "Spinning");
    }

    public Command shooterSpinning() {
        return setPattern("Shooter", new SpinPattern(new Color(57, 32, 144)), "Spinning");
    }

    public Command shooterAtSpeed() {
        return setPattern("Shooter", new FlashPattern(new Color(57, 32, 144), .5), "At Speed");
    }

    public Command grabbedPiece() {
        return setPattern("Intake", Color.kGreen, "Green");
    }

    public Command atPreset() {
        return setPattern("Arm Rotate", new Color(255, 147, 69), "At Preset");
    }

    public Command toPreset() {
        return setPattern("Arm Rotate", new FlashPattern(new Color(112, 255, 248), 0.5), "To preset");
    }

    public Command stowing() {
        return setPattern("Arm Rotate", new Color(255, 82, 174), "Stowing");
    }

    public Command coopButton() {
        return setPattern("HP signal", new Color(255, 191, 0), "Activate Coop");
    }

    public Command ampActivate() {
        return setPattern("HP signal", new Color(133, 19, 79), "Activate AMP");
    }

    public Command visonAligned() {
        return setPattern("Vison", new FlashPattern(new Color(48, 102, 53), 0.5), "To preset");
    }

    public Command awesome() {
        return setPattern("underglow", new Color(255, 32, 82), "AWESOME");
    }

    public Command flash() {
        return setGlobalPattern(new NewFlashPattern(new Color(255, 32, 82), .5));
    }

    // Auto Leds

    public Command toPiece() {
        return setPattern("Auto", new Color(253, 141, 13), "To Piece");
    }

    public Command toSpeaker() {
        return setPattern("Auto", new Color(242, 253, 59), "To Piece");
    }

    public Command atSpeaker() {
        return setPattern("Auto", new Color(253, 63, 236), "To Piece");
    }

}