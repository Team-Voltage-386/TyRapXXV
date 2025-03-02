// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.Subsystems.AlgaeGrabberSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.RangeSensor;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Commands.AlgaeIntake;
import frc.robot.Commands.CenterOnTag;
import frc.robot.Commands.Drive;
import frc.robot.Commands.DriveDistance;
import frc.robot.Commands.DriveLeftOrRight;
import frc.robot.Commands.DriveOffset;
import frc.robot.Commands.EjectAlgae;
import frc.robot.Commands.ElevatorJoystick;
import frc.robot.Commands.MoveStinger;
import frc.robot.Commands.ResetOdoCommand;
import frc.robot.Commands.StopDrive;
import org.json.simple.parser.ParseException;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);
    private final Drivetrain m_swerve;
    private final Limelight m_Limelight;
    // remember to set this to final, commented out range code bc robot doesnt have
    // canrange yet
    private RangeSensor m_range;
    private final AlgaeGrabberSubsystem m_algae;
    private final ClimberSubsystem m_climber;
    private final SendableChooser<String> autoChooser;
    protected final ElevatorSubsystem m_elevator;
    protected final CoralSubsystem m_coral;

    private ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition Tab");
    private GenericEntry m_xVelEntry = m_competitionTab.add("Chassis X Vel", 0).getEntry();
    private GenericEntry m_yVelEntry = m_competitionTab.add("Chassis Y Vel", 0).getEntry();
    private GenericEntry m_gyroAngle = m_competitionTab.add("Gyro Angle", 0).getEntry();
    private GenericEntry m_currentRange = m_competitionTab.add("Range", 0).getEntry();
    private GenericEntry m_commandedXVel = m_competitionTab.add("CommandedVX", 0).getEntry();
    private GenericEntry m_commandedYVel = m_competitionTab.add("CommandedVY", 0).getEntry();
    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    private SwerveModuleSB[] mSwerveModuleTelem;

    Command m_driveCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        this.m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(-90));
        this.m_swerve = new Drivetrain(m_gyro);

        SwerveModuleSB[] swerveModuleTelem = {
                new SwerveModuleSB("FR", m_swerve.getFrontRightSwerveModule(), m_competitionTab),
                new SwerveModuleSB("FL", m_swerve.getFrontLeftSwerveModule(), m_competitionTab),
                new SwerveModuleSB("BR", m_swerve.getBackRightSwerveModule(), m_competitionTab),
                new SwerveModuleSB("BL", m_swerve.getBackLeftSwerveModule(), m_competitionTab) };
        mSwerveModuleTelem = swerveModuleTelem;

        this.m_Limelight = new Limelight();
        this.m_Limelight.setLimelightPipeline(2);
        this.m_algae = new AlgaeGrabberSubsystem(NetworkTableInstance.getDefault());
        this.m_climber = new ClimberSubsystem(m_swerve.getBackLeftSwerveModule().getTurnMotor().getAbsoluteEncoder(), NetworkTableInstance.getDefault());

        // this.m_range = new RangeSensor(0);
        this.m_elevator = new ElevatorSubsystem(NetworkTableInstance.getDefault());
        this.m_coral = new CoralSubsystem(NetworkTableInstance.getDefault());

        // Xbox controllers return negative values when we push forward.
        this.m_driveCommand = new Drive(m_swerve);
        this.m_swerve.setDefaultCommand(this.m_driveCommand);

        autoChooser = new SendableChooser<>(); // Default auto will be `Commands.none()'

        configurePathPlanner();
        autoChooser.setDefaultOption("DO NOTHING!", "NO AUTO");
        m_competitionTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(7, 0);

        m_competitionTab.add("Drivetrain", this.m_swerve);

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        Controller.kDriveController.y().onTrue((new ResetOdoCommand(m_swerve)));
        Controller.kDriveController.rightBumper()
                .onTrue(this.m_swerve.setFieldRelativeCommand(false))
                .onFalse(this.m_swerve.setFieldRelativeCommand(true));

        // Test only to manipulate elevator via left manipulator joystick
        Controller.kManipulatorController.rightTrigger().whileTrue(new ElevatorJoystick(m_elevator));

        Controller.kDriveController.leftBumper().onTrue(m_swerve.setDriveMultCommand(0.5))
                .onFalse(m_swerve.setDriveMultCommand(1));

        // Test commands for centering on tag
        Controller.kDriveController.a().onTrue(new DriveOffset(m_swerve, m_Limelight, false));
        Controller.kDriveController.b().onTrue(new DriveOffset(m_swerve, m_Limelight, true));
        Controller.kDriveController.x().onTrue(new DriveDistance(m_swerve,
            () -> (m_Limelight.getzDistanceMeters() - Constants.Offsets.cameraOffsetFromFrontBumber) + 0.02, 0));
        
        Controller.kDriveController.leftTrigger().whileTrue(new EjectAlgae(m_algae));
        Controller.kDriveController.rightTrigger().whileTrue(new AlgaeIntake(m_algae)); // when disabling robot make sure algae is up

        Controller.kDriveController.povUp().onTrue(m_elevator.runOnce(() -> m_elevator.levelUp()));
        Controller.kDriveController.povDown().onTrue(m_elevator.runOnce(() -> m_elevator.levelDown()));
        
        Controller.kDriveController.povLeft().onTrue(new DriveLeftOrRight(m_swerve, m_Limelight, true));
        Controller.kDriveController.povRight().onTrue(new DriveLeftOrRight(m_swerve, m_Limelight, false));

        //Controller.kDriveController.leftBumper().whileTrue(m_coral.runOnce(() -> m_coral.setVoltageTest(0.3)));
        //Controller.kDriveController.leftBumper().onFalse(m_coral.runOnce(() -> m_coral.setVoltageTest(0.0)));
        //Controller.kDriveController.rightBumper().whileTrue(m_coral.runOnce(() -> m_coral.setVoltageTest(-0.3)));
        //Controller.kDriveController.rightBumper().onFalse(m_coral.runOnce(() -> m_coral.setVoltageTest(0.0)));

        Controller.kManipulatorController.povLeft().whileTrue(new MoveStinger(m_climber, true));
        Controller.kManipulatorController.povRight().whileTrue(new MoveStinger(m_climber, false));
        Controller.kManipulatorController.leftBumper()
                .onTrue(m_climber.runOnce(() -> m_climber.toggleGrabArms()));
        Controller.kManipulatorController.back()
                .onTrue(m_climber.runOnce(() -> m_climber.toggleClimbMode()));
        
    }

    public Drivetrain getDrivetrain() {
        return this.m_swerve;
    }

    private void configurePathPlanner() {
        autoChooser.addOption("Starting2Reef2", "Starting2Reef2");
        autoChooser.addOption("DriveForward", "DriveForward");
        autoChooser.addOption("OnePieceAuto", "OnePieceAuto");
        autoChooser.addOption("Player1Reef1", "Player1Reef1");
        // For multi-step, create name to be name of multi-step, then have object be the name of the first step
        // MultiStep example below
        // autoChooser.addOption("MultiStepRight", "Starting2Reef2");
    }

    public void startAutonomous() {
        String auto = autoChooser.getSelected();
        SequentialCommandGroup start = new SequentialCommandGroup(getAutonomousCommand(autoChooser
                .getSelected()));
        if (auto.equals("Player1Reef1")) { // For testing
            start = new SequentialCommandGroup(getAutonomousCommand(autoChooser.getSelected()),
                    new DriveOffset(m_swerve, m_Limelight, true, 19)); // Add Elevator to L4 and score piece
        }else if(auto.equals("OnePieceAuto")){ // Permanent choice
            start = new SequentialCommandGroup(getAutonomousCommand(autoChooser
                    .getSelected()),
                new DriveOffset(m_swerve, m_Limelight, false)); // Add Elevator to L4 and score piece
        } /*else if (auto.equals("Starting2Reef2")) {
            // MultiStep example/template below
            start = new SequentialCommandGroup(getAutonomousCommand(autoChooser
                    .getSelected()),
                    new DriveOffset(m_swerve, m_Limelight, false), // Add Elevator to L4 and score piece
                    getAutonomousCommand("Reef2Player1"), new CenterOnTag(m_swerve, m_Limelight), // Collect coral here
                    getAutonomousCommand("Player1Reef1"), new DriveOffset(m_swerve, m_Limelight, false), // Add Elevator to L4 and score piece
                    getAutonomousCommand("Reef1Player1"), new CenterOnTag(m_swerve, m_Limelight), // Collect coral here
                    getAutonomousCommand("Player1Reef1"), new DriveOffset(m_swerve, m_Limelight, true), // Add Elevator to L4 and score piece
                    getAutonomousCommand("Reef1Player1"), new CenterOnTag(m_swerve, m_Limelight), // Collect coral here
                    getAutonomousCommand("Player1Reef6"), new DriveOffset(m_swerve, m_Limelight, false) // Add Elevator to L4 and score piece
                    );
        }*/
        start.schedule();
    }

    public Command getAutonomousCommand(String pathName) {
        if (autoChooser.getSelected().equals("NO AUTO")) {
            return Commands.none();
        }
        PathPlannerPath path;
        System.out.println("getAutoCommand building auto for " + autoChooser.getSelected());
        try {
            path = PathPlannerPath.fromPathFile(autoChooser.getSelected());
            Optional<Pose2d> pose = path.getStartingHolonomicPose();
            if (pose.isPresent()) {
                m_swerve.resetStartingPose(pose.get());
                System.out.println(pose.get());
            } else {
                System.out.println("Error getting PathPlanner pose");
            }
            return AutoBuilder.followPath(path);
        } catch (FileVersionException | IOException | ParseException e) {
            System.err.println("Error loading PathPlanner path");
            e.printStackTrace();
        }
        return new StopDrive(m_swerve);
    }

    public void setTeleDefaultCommand() {
        if (this.m_swerve.getDefaultCommand() == null) {
            this.m_swerve.setDefaultCommand(this.m_driveCommand);
        }
    }

    public void setAutoDefaultCommand() {
        if (this.m_swerve.getDefaultCommand() == null) {
            this.m_swerve.setDefaultCommand(this.m_driveCommand);
        }
    }

    public void clearDefaultCommand() {
        this.m_swerve.removeDefaultCommand();
    }

    public void updateConstants() {
        this.m_elevator.updateConstants();
    }

    public void reportTelemetry() {
        m_xVelEntry.setDouble(m_swerve.getChassisSpeeds().vxMetersPerSecond);
        m_yVelEntry.setDouble(m_swerve.getChassisSpeeds().vyMetersPerSecond);
        m_gyroAngle.setDouble(m_swerve.getGyroYawRotation2d().getDegrees());
        for (SwerveModuleSB sb : mSwerveModuleTelem) {
            sb.update();
        }
        SwerveModuleState[] states = {
                m_swerve.getBackLeftSwerveModule().getState(),
                m_swerve.getBackRightSwerveModule().getState(),
                m_swerve.getFrontLeftSwerveModule().getState(),
                m_swerve.getFrontRightSwerveModule().getState() };
        publisher.set(states);
        // m_currentRange.setDouble(m_range.getRange());
        ChassisSpeeds commandedSpeeds = m_swerve.getCommandeChassisSpeeds();
        m_commandedXVel.setDouble(commandedSpeeds.vxMetersPerSecond);
        m_commandedYVel.setDouble(commandedSpeeds.vyMetersPerSecond);
    }
}
