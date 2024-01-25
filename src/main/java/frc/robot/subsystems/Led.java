package frc.robot.subsystems;

import com.chopshop166.chopshoplib.leds.LEDSubsystem;
import com.chopshop166.chopshoplib.leds.patterns.AlliancePattern;
import com.chopshop166.chopshoplib.leds.patterns.ColdFirePattern;
import com.chopshop166.chopshoplib.leds.patterns.SolidColorPattern;
import com.chopshop166.chopshoplib.leds.patterns.SpinPattern;
import com.chopshop166.chopshoplib.maps.LedMap;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class Led extends LEDSubsystem {

    public Led(LedMap map) {
        super(map);
        // This one is length / 2 because the buffer has a mirrored other half
        ledBuffer.setPattern("Fire", new ColdFirePattern(ledBuffer.getLength() / 2));
    }

    public Command colorAlliance() {
        return setPattern("Alliance", new AlliancePattern(), "Alliance");
    }

    public Command resetColor() {
        return setGlobalPattern(new SolidColorPattern(new Color(201, 198, 204)));
    }

    public Command intakeSpinning() {
        return setPattern("Intake", new SpinPattern(), "Spinning");
    }

    public Command shooterSpinning() {
        return setPattern("Shooter", new SpinPattern(), "Spinning");
    }

}