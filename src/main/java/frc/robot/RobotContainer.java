// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.Subsystems.AlgaeGrabberSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.RangeSensor;
import frc.robot.Utils.SafeableSubsystem;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Commands.AlgaeIntake;
import frc.robot.Commands.Drive;
import frc.robot.Commands.DriveDistance;
import frc.robot.Commands.DriveOffset;
import frc.robot.Commands.EjectAlgae;
import frc.robot.Commands.EjectCoral;
import frc.robot.Commands.ElevatorJoystick;
import frc.robot.Commands.MoveCoralManipulator;
import frc.robot.Commands.MoveStinger;
import frc.robot.Commands.ResetOdoCommand;
import frc.robot.Commands.StopDrive;

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
    private SafeableSubsystem[] m_safeable;
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

        // this.m_range = new RangeSensor(0);
        this.m_elevator = new ElevatorSubsystem(NetworkTableInstance.getDefault());
        this.m_coral = new CoralSubsystem(NetworkTableInstance.getDefault());

        // Xbox controllers return negative values when we push forward.
        this.m_driveCommand = new Drive(m_swerve);
        this.m_swerve.setDefaultCommand(this.m_driveCommand);

        SafeableSubsystem[] m_safeable = { m_elevator, m_algae, m_coral };
        this.m_safeable = m_safeable;

        this.m_climber = new ClimberSubsystem(m_swerve.getBackLeftSwerveModule().getTurnMotor().getAbsoluteEncoder(),
                NetworkTableInstance.getDefault(), m_safeable);

        autoChooser = new SendableChooser<>(); // Default auto will be `Commands.none()'

        configurePathPlanner();
        autoChooser.setDefaultOption("DO NOTHING!", "NO AUTO");
        m_competitionTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(7, 0);
        m_competitionTab.add("Drivetrain", this.m_swerve);

        NamedCommands.registerCommand("StopDrive", new StopDrive(m_swerve));

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

        Controller.kManipulatorController.rightTrigger().and(m_climber::isCoralMode)
                .whileTrue(new ElevatorJoystick(m_elevator));

        Controller.kDriveController.leftBumper().onTrue(m_swerve.setDriveMultCommand(0.5))
                .onFalse(m_swerve.setDriveMultCommand(1));
        Controller.kDriveController.a().onTrue(new DriveOffset(m_swerve, m_Limelight, false));
        Controller.kDriveController.b().onTrue(new DriveOffset(m_swerve, m_Limelight, true));
        Controller.kDriveController.x().onTrue(new DriveDistance(m_swerve,
                () -> m_Limelight.getzDistanceMeters() - 0.1, 0));

        Controller.kDriveController.leftTrigger().and(m_climber::isCoralMode).whileTrue(new EjectAlgae(m_algae));
        Controller.kDriveController.rightTrigger().and(m_climber::isCoralMode).whileTrue(new AlgaeIntake(m_algae));

        // when disabling robot, make
        // sure grabber isnt extended
        // Controller.kDriveController.leftBumper().onTrue(new DriveRange(m_swerve, ()
        // -> 0.5, () -> m_range.getRange(), 90, 0.2));

        Controller.kManipulatorController.povUp().and(m_climber::isCoralMode)
                .onTrue(m_elevator.runOnce(() -> m_elevator.levelUp()));
        Controller.kManipulatorController.povDown().and(m_climber::isCoralMode)
                .onTrue(m_elevator.runOnce(() -> m_elevator.levelDown()));

        Controller.kManipulatorController.povLeft().whileTrue(new MoveStinger(m_climber, true));
        Controller.kManipulatorController.povRight().whileTrue(new MoveStinger(m_climber, false));

        Controller.kDriveController.povUp().and(m_climber::isCoralMode)
                .whileTrue(new MoveCoralManipulator(m_coral, true));
        Controller.kDriveController.povDown().and(m_climber::isCoralMode)
                .whileTrue(new MoveCoralManipulator(m_coral, false));
        Controller.kDriveController.x().and(m_climber::isCoralMode).whileTrue(new EjectCoral(m_coral));

        Controller.kManipulatorController.leftBumper()
                .onTrue(m_climber.runOnce(() -> m_climber.toggleGrabArms()));
        Controller.kManipulatorController.back()
                .onTrue(m_climber.runOnce(() -> m_climber.toggleClimbMode()));
    }

    public Drivetrain getDrivetrain() {
        return this.m_swerve;
    }

    private void configurePathPlanner() {
        autoChooser.addOption("Vision Test", "Vision Test");
        autoChooser.addOption("Drive Straight", "Drive Straight");
        autoChooser.addOption("SwerveTestAuto25", "SwerveTestAuto25");
        autoChooser.addOption("StraightForward", "StraightForward");
    }

    public Command getAutonomousCommand() {
        if (autoChooser.getSelected().equals("NO AUTO")) {
            return Commands.none();
        }
        System.out.println("getAutoCommand building auto for " + autoChooser.getSelected());
        return AutoBuilder.buildAuto(autoChooser.getSelected());
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
