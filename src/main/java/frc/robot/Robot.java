// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.RobotMap;

public class Robot extends CommandRobot {

    private RobotMap map /* = getRobotMap(RobotMap.class, new RobotMap()) */;
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private ButtonXboxController copilotController = new ButtonXboxController(1);

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    @Autonomous(name = "No Auto", defaultAuto = true)
    public Command noAuto = Commands.none();

    @Override
    public void robotInit() {
        super.robotInit();

        Logger.recordMetadata("ProjectName", "FRC-2024"); // Set a metadata value
        map.setupLogging();
        if (!isReal()) {
            setUseTiming(false); // Run as fast as possible
        }
        // Start logging! No more data receivers, replay sources, or metadata values may
        // be added.
        Logger.start();
    }

    @Override
    public void configureButtonBindings() {
    }

    @Override
    public void populateDashboard() {
    }

    @Override
    public void setDefaultCommands() {
    }
}
