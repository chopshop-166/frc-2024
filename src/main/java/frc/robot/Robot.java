// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Drive;

public class Robot extends CommandRobot {

    private RobotMap map = getRobotMap(RobotMap.class, new RobotMap());
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private ButtonXboxController copilotController = new ButtonXboxController(1);

    private Drive drive = new Drive(map.getDriveMap());

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    @Autonomous(name = "No Auto", defaultAuto = true)
    public Command noAuto = Commands.none();

    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

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
        driveController.back().onTrue(drive.resetGyroCommand());
        // Magic numbers for auto testing
        driveController.start().onTrue(drive.setPose(new Pose2d(1.5, 3.5, Rotation2d.fromDegrees(0))));
    }

    @Override
    public void populateDashboard() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }

    @Override
    public void setDefaultCommands() {
        drive.setDefaultCommand(
                drive.drive(() -> -driveController.getLeftX(), () -> -driveController.getLeftY(),
                        () -> -driveController.getRightX()));
    }
}
