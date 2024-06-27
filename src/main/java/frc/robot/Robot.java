// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleUnaryOperator;

import org.littletonrobotics.junction.Logger;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.RobotUtils;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.maps.RobotMap;
import frc.robot.maps.subsystems.ArmRotateMap.ArmPresets;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.Speeds;
import frc.robot.subsystems.Undertaker;

public class Robot extends CommandRobot {

    private RobotMap map = getRobotMap(RobotMap.class, new RobotMap());
    private ButtonXboxController driveController = new ButtonXboxController(0);
    private ButtonXboxController copilotController = new ButtonXboxController(1);

    // Helpers
    final DoubleUnaryOperator driveScaler = getScaler(0.45, 0.25);

    private Drive drive = new Drive(map.getDriveMap(), () -> {
        return driveScaler.applyAsDouble(-driveController.getLeftX());
    }, () -> {
        return driveScaler.applyAsDouble(-driveController.getLeftY());
    }, () -> {
        return driveScaler.applyAsDouble(-driveController.getRightX());
    });
    private Intake intake = new Intake(map.getIntakeMap());
    private Shooter shooter = new Shooter(map.getShooterMap());
    private Led led = new Led(map.getLedMap());
    private ArmRotate armRotate = new ArmRotate(map.getArmRotateMap());
    private Undertaker undertaker = new Undertaker(map.getUndertakerMap());
    private CommandSequences commandSequences = new CommandSequences(drive, intake, shooter, led, armRotate,
            undertaker);

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    public void registerNamedCommands() {
        // Register named commands. These are all set as the reset gyro command, please
        // input actual commands once all subsystems are merged with main. Do this
        // correctly, as the names are already in PathPlanner. We should probably make
        // sequences in this code if needed (like adding arm rotation to shoot) so that
        // PathPlanner doesn't get too complicated. You might need to add wait
        // commands into PathPlanner.
        NamedCommands.registerCommand("Intake Game Piece", commandSequences.moveAndIntake());
        NamedCommands.registerCommand("Intake Contingency", commandSequences.moveAndIntakeContingency());
        NamedCommands.registerCommand("Shoot Game Piece - Subwoofer", commandSequences.scoreSpeakerAuto());
        NamedCommands.registerCommand("Shoot Game Piece - Podium", commandSequences.scoreSpeakerPodiumAuto());
        NamedCommands.registerCommand("Rotate Arm Sub",
                commandSequences.armRotatePreset(ArmPresets.SCORE_SPEAKER_SUBWOOFER));
        NamedCommands.registerCommand("Stop Shooter", shooter.setSpeed(Speeds.OFF));
        NamedCommands.registerCommand("Intake Contingency Two",
                commandSequences.moveAndIntakeContingencyTwo());
        NamedCommands.registerCommand("Rotate Speaker Pod.",
                commandSequences.armRotatePreset(ArmPresets.SCORE_SPEAKER_PODIUM));
        NamedCommands.registerCommand("Has Game Piece? Rotate.",
                commandSequences.autoGamePieceDetected());
        NamedCommands.registerCommand("Rotate to Intake", commandSequences.rotateToIntake());
    }

    @Autonomous(name = "No Auto", defaultAuto = true)
    public Command noAuto = Commands.none();

    private final SendableChooser<Command> autoChooser;

    public Robot() {
        super();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
    }

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

        led.colorAlliance().schedule();
        DriverStation.silenceJoystickConnectionWarning(true);

    }

    @Override
    public void disabledInit() {
        super.disabledInit();
        led.colorAlliance().schedule();
    }

    @Override
    public void configureButtonBindings() {
        driveController.back().onTrue(drive.resetCmd());
        driveController.leftBumper()
                .whileTrue(drive.robotCentricDrive());

        driveController.y().whileTrue(commandSequences.charge(Speeds.FULL_SPEED, ArmPresets.SHUTTLE))
                .onFalse(commandSequences.release());

        driveController.a().onTrue(commandSequences.moveAndIntake());

        copilotController.back().onTrue(intake.safeStateCmd().andThen(armRotate.safeStateCmd()));
        copilotController.start().onTrue(shooter.setSpeed(Speeds.OFF));
        copilotController.a().onTrue(commandSequences.moveAndIntake());
        copilotController.b()
                .whileTrue(commandSequences.charge(Speeds.SUBWOOFER_SHOT, ArmPresets.SCORE_SPEAKER_SUBWOOFER))
                .onFalse(commandSequences.release());
        copilotController.y().whileTrue(commandSequences.charge(Speeds.SHUTTLE_SHOT, ArmPresets.SHUTTLE))
                .onFalse(commandSequences.release());
        copilotController.x().whileTrue(intake.spinOut().alongWith(undertaker.spinOut()));
        copilotController.rightStick().whileTrue(intake.feedShooter());
        copilotController.rightBumper().whileTrue(commandSequences.charge(Speeds.AMP_SPEED, ArmPresets.SCORE_AMP))
                .onFalse(commandSequences.release());
        copilotController.leftBumper()
                .whileTrue(commandSequences.charge(Speeds.PODIUM_SHOT, ArmPresets.SCORE_SPEAKER_PODIUM))
                .onFalse(commandSequences.release());
        copilotController.povUp().onTrue(commandSequences.moveToAmp());
        copilotController.povDown().onTrue(commandSequences.shooterSpeed(Speeds.FULL_SPEED));
        copilotController.povRight().onTrue(commandSequences.armRotatePreset(ArmPresets.SCORE_SPEAKER_SUBWOOFER));
        copilotController.povLeft().onTrue(commandSequences.stow());
    }

    @Override
    public void populateDashboard() {
        var pitTestTab = Shuffleboard.getTab("Pit Test");
        pitTestTab.add("Drive Backward", drive.moveInDirection(0, -1, 3)).withPosition(1, 2);
        pitTestTab.add("Drive Left", drive.moveInDirection(1, 0, 3)).withPosition(0, 1);
        pitTestTab.add("Drive Right", drive.moveInDirection(-1, 0, 3)).withPosition(2, 1);
        pitTestTab.add("Drive Forward", drive.moveInDirection(0, 1, 3)).withPosition(1, 0);
        pitTestTab.add("Drive Forward Faster", drive.moveInDirection(0, 2, 3)).withPosition(4, 1);
        pitTestTab.add("Stop", drive.moveInDirection(0, 0, 0)).withPosition(1, 1);
        pitTestTab.add("Foward Right", drive.moveInDirection(1, -1, 3)).withPosition(2, 0);
        pitTestTab.add("Foward Left", drive.moveInDirection(-1, 1, 3)).withPosition(0, 0);
        pitTestTab.add("Back Right", drive.moveInDirection(1, 1, 3)).withPosition(2, 2);
        pitTestTab.add("Back Left", drive.moveInDirection(-1, 1, 3)).withPosition(0, 2);
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
        armRotate.setDefaultCommand(armRotate.move(RobotUtils.deadbandAxis(.1, () -> -copilotController.getLeftY())));
    }

    public DoubleUnaryOperator getScaler(double leftRange, double rightRange) {
        return speed -> {
            double leftTrigger = driveController.getLeftTriggerAxis();
            double rightTrigger = driveController.getRightTriggerAxis();
            double modifier = (rightRange * rightTrigger) - (leftRange * leftTrigger) + 0.75;
            return modifier * speed;
        };
    }
}