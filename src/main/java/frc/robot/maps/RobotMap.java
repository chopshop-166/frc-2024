package frc.robot.maps;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.chopshop166.chopshoplib.maps.SwerveDriveMap;

import frc.robot.maps.subsystems.ArmRotateMap;
import com.chopshop166.chopshoplib.maps.LedMap;

import frc.robot.maps.subsystems.IntakeMap;
import frc.robot.maps.subsystems.ShooterMap;
import frc.robot.maps.subsystems.UndertakerMap;

public class RobotMap {

    public SwerveDriveMap getDriveMap() {
        return new SwerveDriveMap();
    }

    public ArmRotateMap getArmRotateMap() {
        return new ArmRotateMap();
    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public ShooterMap getShooterMap() {
        return new ShooterMap();
    }

    public LedMap getLedMap() {
        return new LedMap();
    }

    public UndertakerMap getUndertakerMap() {
        return new UndertakerMap();
    }

    public ClawMap getClawMap() {
        return new ClawMap();
    }

    public void setupLogging() {
        // Pull the replay log from AdvantageScope (or prompt the user)
        String logPath = LogFileUtil.findReplayLog();
        // Read replay log
        Logger.setReplaySource(new WPILOGReader(logPath));
        // Save outputs to a new log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
}
