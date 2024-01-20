package frc.robot.maps;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.maps.subsystems.ArmRotateMap;
import frc.robot.maps.subsystems.IntakeMap;
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
    public void setupLogging() {
        // Pull the replay log from AdvantageScope (or prompt the user)
        String logPath = LogFileUtil.findReplayLog();
        // Read replay log
        Logger.setReplaySource(new WPILOGReader(logPath));
        // Save outputs to a new log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
}