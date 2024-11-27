package frc.robot.maps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.leds.SegmentConfig;
import com.chopshop166.chopshoplib.maps.LedMap;
import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.pneumatics.RevDSolenoid;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

@RobotMapFor("00:80:2F:19:78:A9")
public class Thomas extends RobotMap {

    @Override
    public LedMap getLedMap() {
        var result = new LedMap(0, 150);
        var leds = result.ledBuffer;

        SegmentConfig underglow = leds.segment(150).tags("underglow", "Shooter", "Arm Rotate", "Intake", "HP signal",
                "Vision", "Fire", "Auto", "Alliance");
        return result;
    }

    @Override
    public ClawMap get ClawMap() {
        RevDSolenoid piston = new RevDSolenoid(0, 1);
        return new ClawMap(piston);
    }

    @Override
    public void setupLogging() {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        Logger.recordMetadata("RobotMap", this.getClass().getSimpleName());
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
}
