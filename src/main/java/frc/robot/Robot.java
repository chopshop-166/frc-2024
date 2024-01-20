// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.Henry;
import frc.robot.maps.RobotMap;
import frc.robot.maps.Valkyrie;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class Robot extends CommandRobot {

    // private RobotMap map = getRobotMap(RobotMap.class, new RobotMap());
    private RobotMap map = new Valkyrie();
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private ButtonXboxController copilotController = new ButtonXboxController(1);

    private Drive drive = new Drive(map.getDriveMap());
    private Intake intake = new Intake(map.getIntakeMap());

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    @Autonomous(name = "No Auto", defaultAuto = true)
    public Command noAuto = Commands.none();

    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    @Override
    public void robotInit() {
        super.robotInit();

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

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
        driveController.start().onTrue(drive.setPoseCommand(new Pose2d(2, 7, Rotation2d.fromDegrees(0))));
        driveController.a().whileTrue(drive.robotCentricDrive(() -> -driveController.getLeftX(), () -> -driveController.getLeftY(),
                        () -> -driveController.getRightX()));
    }

    @Override
    public void populateDashboard() {
        Shuffleboard.getTab("Pit Test");
        Shuffleboard.getTab("Pit Test").add("Drive Backward",
                drive.moveInDirection(0, -1, 3)).withPosition(1, 2);
        Shuffleboard.getTab("Pit Test").add("Drive Left",
                drive.moveInDirection(-1, 0, 3)).withPosition(0, 1);
        Shuffleboard.getTab("Pit Test").add("Drive Right",
                drive.moveInDirection(1, 0, 3)).withPosition(2, 1);
        Shuffleboard.getTab("Pit Test").add("Drive Forward",
                drive.moveInDirection(0, 1, 3)).withPosition(1, 0);
        Shuffleboard.getTab("Pit Test").add("Stop", drive.moveInDirection(0, 0,
                0)).withPosition(1, 1);
        Shuffleboard.getTab("Pit Test").add("Foward Right",
                drive.moveInDirection(1, -1, 3)).withPosition(2, 0);
        Shuffleboard.getTab("Pit Test").add("Foward Left",
                drive.moveInDirection(-1, 1, 3)).withPosition(0, 0);
        Shuffleboard.getTab("Pit Test").add("Back Right", drive.moveInDirection(1,
                1, 3)).withPosition(2, 2);
        Shuffleboard.getTab("Pit Test").add("Back Left", drive.moveInDirection(-1,
                1, 3)).withPosition(0, 2);
        Shuffleboard.getTab("AutoBuilder").add("Auto", autoChooser);
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