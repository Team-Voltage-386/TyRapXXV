// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.*;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.Subsystems.AlgaeGrabberSubsystem;
import frc.robot.Subsystems.LightSubsystem;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorLevel;
import frc.robot.Subsystems.LightSubsystem.AlgaeState;
import frc.robot.Subsystems.Limelight;
import frc.robot.Utils.SafeableSubsystem;
import frc.robot.Subsystems.CoralSubsystem;
import frc.robot.Commands.AlgaeIntake;
import frc.robot.Commands.Drive;
import frc.robot.Commands.DriveDistance;
import frc.robot.Commands.DriveDistance2;
import frc.robot.Commands.DriveLeftOrRight;
import frc.robot.Commands.DriveFixedVelocity;
import frc.robot.Commands.DriveOffset;
import frc.robot.Commands.EjectAlgae;
import frc.robot.Commands.EjectCoral;
import frc.robot.Commands.GoToFlagLevel;
import frc.robot.Commands.MoveCoralManipulator;
import frc.robot.Commands.MoveStinger;
import frc.robot.Commands.ResetOdoCommand;
import frc.robot.Commands.RotateWheels;
import frc.robot.Commands.StationaryWait;
import frc.robot.Commands.StopCoral;
import frc.robot.Commands.StopDrive;
import org.json.simple.parser.ParseException;
import frc.robot.Commands.RumbleDrive;
import frc.robot.Commands.StopElevator;
import frc.robot.Commands.GoToLevel;

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
        private SafeableSubsystem[] m_safeable;
        private final AlgaeGrabberSubsystem m_algae;
        private final ClimberSubsystem m_climber;
        private final SendableChooser<String> autoChooser;
        protected final ElevatorSubsystem m_elevator;
        protected final CoralSubsystem m_coral;
        protected final LightSubsystem m_leds;

        private ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition Tab");
        private GenericEntry m_xVelEntry = m_competitionTab.add("Chassis X Vel", 0).getEntry();
        private GenericEntry m_yVelEntry = m_competitionTab.add("Chassis Y Vel", 0).getEntry();
        private GenericEntry m_gyroAngle = m_competitionTab.add("Gyro Angle", 0).getEntry();
        private GenericEntry m_commandedXVel = m_competitionTab.add("CommandedVX", 0).getEntry();
        private GenericEntry m_commandedYVel = m_competitionTab.add("CommandedVY", 0).getEntry();
        private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
        protected GenericEntry m_driveP = m_competitionTab.add("Drive P Val", DrivetrainConstants.drivePID[0])
                        .getEntry();
        protected GenericEntry m_driveFFStatic = m_competitionTab
                        .add("Drive FF Static", DrivetrainConstants.driveFeedForward[0]).getEntry();
        protected GenericEntry m_driveFFVel = m_competitionTab
                        .add("Drive FF Vel", DrivetrainConstants.driveFeedForward[1])
                        .getEntry();
        protected GenericEntry m_driveAccel = m_competitionTab.add("Drive FF Accel", 0.0).getEntry();
        protected GenericEntry m_turnP = m_competitionTab.add("Turn P Val", DrivetrainConstants.turnPID[0]).getEntry();
        protected GenericEntry m_turnI = m_competitionTab.add("Turn I Val", DrivetrainConstants.turnPID[1]).getEntry();
        protected GenericEntry m_turnFFStatic = m_competitionTab
                        .add("Turn FF Static", DrivetrainConstants.turnFeedForward[0]).getEntry();
        protected GenericEntry m_turnFFVel = m_competitionTab.add("Turn FF Vel", DrivetrainConstants.turnFeedForward[1])
                        .getEntry();

        private GenericEntry m_fixedSpeed = m_competitionTab.add("Fixed Speed", 0).getEntry();
        private SwerveModuleSB[] mSwerveModuleTelem;

        protected UsbCamera climbCamera;

        Command m_driveCommand;

        boolean bindingsConfigured = false;

        private enum CommandSelector {
                LOW, HIGH, NULL;
        }
        

        SequentialCommandGroup m_scoreCancel;
        SequentialCommandGroup m_scoreLeft;
        SequentialCommandGroup m_scoreRight;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                this.m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(0));
                this.m_gyro.resetSignalFrequencies();
                
                this.m_swerve = new Drivetrain(m_gyro);

                SwerveModuleSB[] swerveModuleTelem = {
                                new SwerveModuleSB("FR", m_swerve.getFrontRightSwerveModule(), m_competitionTab),
                                new SwerveModuleSB("FL", m_swerve.getFrontLeftSwerveModule(), m_competitionTab),
                                new SwerveModuleSB("BR", m_swerve.getBackRightSwerveModule(), m_competitionTab),
                                new SwerveModuleSB("BL", m_swerve.getBackLeftSwerveModule(), m_competitionTab) };
                mSwerveModuleTelem = swerveModuleTelem;

                this.m_Limelight = new Limelight();
                this.m_Limelight.setLimelightPipeline(LimelightConstants.defaultPipeline);
                climbCamera = CameraServer.startAutomaticCapture(); // Start USB webcam capture for climb
                climbCamera.setFPS(18);
                climbCamera.setPixelFormat(PixelFormat.kMJPEG);
                this.m_algae = new AlgaeGrabberSubsystem(NetworkTableInstance.getDefault());

                // this.m_range = new RangeSensor(0);
                this.m_elevator = new ElevatorSubsystem(NetworkTableInstance.getDefault());
                this.m_coral = new CoralSubsystem(NetworkTableInstance.getDefault(), m_elevator);

                AddressableLED led = new AddressableLED(0);
                led.setLength(5);
                AddressableLEDBuffer ledBuf = new AddressableLEDBuffer(5);

                this.m_leds = new LightSubsystem(led, ledBuf, m_Limelight, m_coral, m_elevator);

                // Xbox controllers return negative values when we push forward.
                this.m_driveCommand = new Drive(m_swerve);
                this.m_swerve.setDefaultCommand(this.m_driveCommand);

                SafeableSubsystem[] m_safeable = { m_elevator, m_algae, m_coral };
                this.m_safeable = m_safeable;
                this.m_climber = new ClimberSubsystem(
                                m_swerve.getBackLeftSwerveModule().getTurnMotor().getAbsoluteEncoder(),
                                NetworkTableInstance.getDefault(), m_safeable);

                autoChooser = new SendableChooser<>(); // Default auto will be `Commands.none()'

                configurePathPlanner();
                autoChooser.setDefaultOption("DO NOTHING!", "NO AUTO");
                m_competitionTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(7, 0);

                m_competitionTab.add("Drivetrain", this.m_swerve);

                NamedCommands.registerCommand("StopDrive", new StopDrive(m_swerve));

                

                this.m_scoreLeft = new SequentialCommandGroup(
                                new ConditionalCommand(
                                                new ConditionalCommand(
                                                                buildScoreOffsetCommand(true),
                                                                buildScoreBumperedUpCommand(true, 0.15),
                                                                () -> m_Limelight
                                                                                .getzDistanceMeters() > (Offsets.cameraOffsetFromFrontBumber
                                                                                                + 0.1)),
                                                new PrintCommand("level has not been set").andThen(new RumbleDrive(.5)),
                                                () -> (m_elevator.isAnyLevelSet()) && m_leds.canSeeValidTag()));

                this.m_scoreRight = new SequentialCommandGroup(
                                new ConditionalCommand(
                                                new ConditionalCommand(
                                                                buildScoreOffsetCommand(false),
                                                                buildScoreBumperedUpCommand(false, 0.15),
                                                                () -> m_Limelight
                                                                                .getzDistanceMeters() > (Offsets.cameraOffsetFromFrontBumber
                                                                                                + 0.1)),
                                                new PrintCommand("level has not been set").andThen(new RumbleDrive(.5)),
                                                () -> (m_elevator.isAnyLevelSet()) && m_leds.canSeeValidTag()));

                this.m_scoreCancel = new SequentialCommandGroup(
                                m_elevator.runOnce(() -> m_scoreLeft.cancel()),
                                m_elevator.runOnce(() -> m_scoreRight.cancel()),
                                new StopDrive(m_swerve),
                                new StopCoral(m_coral),
                                new StopElevator(m_elevator));
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
        public void configureBindings() {
                if (bindingsConfigured) {
                        return;
                } else {
                        bindingsConfigured = true;
                }

                // DRIVE CONTROLLERS BINDINGS

                // Bumper Buttons for Scoring Sequence
                Controller.kDriveController.rightBumper().and(m_climber::isCoralMode).onTrue(m_scoreRight);

                // Bumper Buttons for Scoring Sequence
                Controller.kDriveController.leftBumper().and(m_climber::isCoralMode).onTrue(m_scoreLeft);

                // Toggle Robot Oriented Drive
                Controller.kDriveController.back()
                                .toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());

                // Cancel Coral Score
                Controller.kDriveController.a().onTrue(m_scoreCancel);

                // reset Field Orient Command
                Controller.kDriveController.y().onTrue((new ResetOdoCommand(m_swerve)));

                // Coral extend and retract
                Controller.kDriveController.x().and(m_climber::isCoralMode).onTrue(m_coral.wristExtendCommand());
                Controller.kDriveController.b().and(m_climber::isCoralMode).onTrue(m_coral.wristRetractCommand());

                // Trigger Buttons for Algae Intake and Eject
                Controller.kDriveController.leftTrigger().and(m_climber::isCoralMode)
                                .whileTrue(new EjectAlgae(m_algae));
                Controller.kDriveController.rightTrigger().and(m_climber::isCoralMode)
                                .whileTrue(new AlgaeIntake(m_algae)); // when disabling robot make
                // sure grabber isnt extended

                // MANIPULATOR CONTROLLER BINDINGS:

                // D-Pad for Elevator Manual Control
                Controller.kManipulatorController.povUp().and(m_climber::isCoralMode)
                                .onTrue(m_elevator.runOnce(() -> m_elevator.manualUp()))
                                .onFalse(m_elevator.runOnce(() -> m_elevator.stopManualMode()));
                Controller.kManipulatorController.povDown().and(m_climber::isCoralMode)
                                .onTrue(m_elevator.runOnce(() -> m_elevator.manualDown()))
                                .onFalse(m_elevator.runOnce(() -> m_elevator.stopManualMode()));

                // D-Pad for Stinger Control
                Controller.kManipulatorController.povLeft().whileTrue(new MoveStinger(m_climber, true));
                Controller.kManipulatorController.povRight().whileTrue(new MoveStinger(m_climber, false));

                // Bumper Buttons
                Controller.kManipulatorController.leftBumper()
                                .onTrue(m_climber.runOnce(() -> m_climber.toggleGrabArms()));

                // Triger Buttons
                Controller.kManipulatorController.rightTrigger().onTrue(new EjectCoral(m_coral));
                Controller.kManipulatorController.rightBumper().and(m_climber::isCoralMode)
                                .onTrue(buildSelectRemoveAlgaeCommand());

                // Back Button and Start button for Climber Mode Toggle
                Controller.kManipulatorController.back()
                                .onTrue(m_climber.runOnce(() -> m_climber.setClimbMode()));
                Controller.kManipulatorController.start()
                                .onTrue(m_climber.runOnce(() -> m_climber.setCoralMode()));

                // X, A, B, Y for Elevator Level Flags
                Controller.kManipulatorController.x()
                                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL1)));
                Controller.kManipulatorController.a()
                                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL2)));
                Controller.kManipulatorController.b()
                                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL3)));
                Controller.kManipulatorController.y()
                                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL4)));

                // TEMPORARY BINDINGS FOR TESTING:
                Controller.kDriveController.povUp().and(m_climber::isCoralMode)
                                .whileTrue(new MoveCoralManipulator(m_coral, true));
                Controller.kDriveController.povDown().and(m_climber::isCoralMode)
                                .whileTrue(new MoveCoralManipulator(m_coral, false));
                Controller.kDriveController.povLeft().and(m_climber::isCoralMode)
                                .onTrue(m_elevator.runOnce(() -> m_elevator.levelDown()));
                Controller.kDriveController.povRight().and(m_climber::isCoralMode)
                                .onTrue(m_elevator.runOnce(() -> m_elevator.levelUp()));
        }

        public void configureTestBindings() {
                if (bindingsConfigured) {
                        return;
                } else {
                        bindingsConfigured = true;
                }

                Controller.kDriveController.y().onTrue((new ResetOdoCommand(m_swerve)));

                Controller.kDriveController.back()
                                .toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());

                // Controller.kManipulatorController.rightTrigger().whileTrue(new
                // ElevatorJoystick(m_elevator));

                Controller.kDriveController.leftBumper().whileTrue(new RotateWheels(m_swerve, 0.0));
                Controller.kDriveController.rightBumper().whileTrue(new RotateWheels(m_swerve, 90.0));

                Controller.kDriveController.a().onTrue(new DriveOffset(m_swerve, m_Limelight, false));
                Controller.kDriveController.b().onTrue(new DriveOffset(m_swerve, m_Limelight, true));
                /*
                 * Controller.kDriveController.x().onTrue(new DriveDistance(m_swerve,
                 * () -> m_Limelight.getzDistanceMeters() - 0.1, 0));
                 */

                // Test commands for centering on tag
                Controller.kDriveController.x().onTrue(new DriveDistance(m_swerve,
                                () -> (m_Limelight.getzDistanceMeters() - Constants.Offsets.cameraOffsetFromFrontBumber)
                                                + 0.02,
                                0));

                Controller.kDriveController.leftTrigger().and(m_climber::isCoralMode)
                                .whileTrue(new EjectAlgae(m_algae));
                Controller.kDriveController.rightTrigger().and(m_climber::isCoralMode)
                                .whileTrue(new AlgaeIntake(m_algae));

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

                Controller.kDriveController.povUp()
                                .whileTrue(new DriveFixedVelocity(m_swerve, 0, () -> m_fixedSpeed.getDouble(0.5)));
                Controller.kDriveController.povRight()
                                .whileTrue(new DriveFixedVelocity(m_swerve, 90, () -> m_fixedSpeed.getDouble(0.5)));
                Controller.kDriveController.povDown()
                                .whileTrue(new DriveFixedVelocity(m_swerve, 180, () -> m_fixedSpeed.getDouble(0.5)));
                Controller.kDriveController.povLeft()
                                .whileTrue(new DriveFixedVelocity(m_swerve, 270, () -> m_fixedSpeed.getDouble(0.5)));
                Controller.kDriveController.x().whileTrue(new EjectCoral(m_coral));

                Controller.kManipulatorController.leftBumper()
                                .onTrue(m_climber.runOnce(() -> m_climber.toggleGrabArms()));
                Controller.kManipulatorController.back()
                                .onTrue(m_climber.runOnce(() -> m_climber.setClimbMode()));
                Controller.kManipulatorController.start()
                                .onTrue(m_climber.runOnce(() -> m_climber.setCoralMode()));

                Controller.kManipulatorController.x()
                                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL1)));
                Controller.kManipulatorController.a()
                                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL2)));
                Controller.kManipulatorController.b()
                                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL3)));
                Controller.kManipulatorController.y()
                                .onTrue(m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.LEVEL4)));
        }

        private CommandSelector select() {
                AlgaeState state = m_leds.determineAlgaeHeight();
                if (state == AlgaeState.LOW){
                        return CommandSelector.LOW;
                }
                else if (state == AlgaeState.HIGH){
                        return CommandSelector.HIGH;
                }
                else return CommandSelector.NULL;
              }

        private Command buildSelectRemoveAlgaeCommand(){
        return new SelectCommand<>(
                Map.ofEntries(
                Map.entry(CommandSelector.LOW, buildRemoveAlgaeCommand(ElevatorLevel.LEVEL3, ElevatorLevel.LEVEL5)),
                Map.entry(CommandSelector.HIGH, buildRemoveAlgaeCommand(ElevatorLevel.LEVEL4, ElevatorLevel.LEVEL6)),
                Map.entry(CommandSelector.NULL, new RumbleDrive(0.5))),
        this::select);
        }

        protected SequentialCommandGroup buildScoreOffsetCommand(boolean isLeft) {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new DriveOffset(m_swerve, m_Limelight, isLeft),
                                                                new DriveDistance2(m_swerve,
                                                                                () -> (m_Limelight.getzDistanceMeters()
                                                                                                - .42),
                                                                                0)
                                                                                .withTimeout(1),
                                                                new StopDrive(m_swerve)),
                                                new GoToFlagLevel(m_elevator)),
                                new EjectCoral(m_coral),
                                new StationaryWait(m_swerve, .4),
                                new ConditionalCommand(
                                        buildSelectRemoveAlgaeCommand(), 
                                        new DriveFixedVelocity(m_swerve, 180, () -> 3.0).withTimeout(0.20),
                                        () -> (shouldRemoveAlgae())),
                                m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND)));
        }

        protected SequentialCommandGroup buildScoreBumperedUpCommand(boolean isLeft, double forwardTimeout) {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                // Raise the elevator to the selected level while in parallel aligning
                                                // left or
                                                // right
                                                new GoToFlagLevel(m_elevator).withTimeout(2.5),
                                                new SequentialCommandGroup(
                                                                new DriveDistance2(m_swerve, () -> 0.15, 0)
                                                                                .withTimeout(forwardTimeout),
                                                                new DriveFixedVelocity(m_swerve, 180, () -> 0.25)
                                                                                .withTimeout(.1),
                                                                new DriveLeftOrRight(m_swerve, m_Limelight, isLeft),
                                                                new StopDrive(m_swerve))),
                                new EjectCoral(m_coral),
                                new StationaryWait(m_swerve, .4),
                                new ConditionalCommand(
                                        buildSelectRemoveAlgaeCommand(),  
                                        new DriveFixedVelocity(m_swerve, 180, () -> 3.0).withTimeout(0.20),
                                        () -> (shouldRemoveAlgae())),
                                m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND)));
        }

        protected SequentialCommandGroup buildScoreOffsetAutoCommand(boolean isLeft) {
                return new SequentialCommandGroup(
                                new PrintCommand("Running offset score routine"),
                                new DriveOffset(m_swerve, m_Limelight, isLeft),
                                new ParallelCommandGroup(
                                            new GoToFlagLevel(m_elevator),
                                            new SequentialCommandGroup(
                                                    new DriveDistance2(m_swerve,
                                                        () -> (m_Limelight.getzDistanceMeters()
                                                                            - .42),
                                                                    0)
                                                                    .withTimeout(1),
                                                    new StopDrive(m_swerve))),
                                new EjectCoral(m_coral),
                                new StationaryWait(m_swerve, .4),
                                new DriveDistance2(m_swerve, () -> 0.1, 180).withTimeout(0.4),
                                new StopDrive(m_swerve),
                                m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND)));
        }

        protected SequentialCommandGroup buildScoreBumperedUpAutoCommand(boolean isLeft, double forwardTimeout) {
                return new SequentialCommandGroup(
                                new PrintCommand("Running drive left right score"),
                                new ParallelCommandGroup(
                                                // Raise the elevator to the selected level while in parallel aligning
                                                // left or
                                                // right
                                                new GoToFlagLevel(m_elevator).withTimeout(2.5),
                                                new SequentialCommandGroup(
                                                                // new DriveDistance(m_swerve, () -> 0.7,
                                                                // 0).withTimeout(forwardTimeout),
                                                                new DriveFixedVelocity(m_swerve, 0, () -> 1.5)
                                                                                .withTimeout(0.5),
                                                                new DriveFixedVelocity(m_swerve, 180, () -> 0.25)
                                                                                .withTimeout(.1),
                                                                new DriveLeftOrRight(m_swerve, m_Limelight, isLeft),
                                                                new StopDrive(m_swerve))),
                                new StopDrive(m_swerve),
                                new EjectCoral(m_coral),
                                new StationaryWait(m_swerve, .5),
                                new DriveDistance2(m_swerve, () -> 0.1, 180).withTimeout(.4),
                                new StopDrive(m_swerve),
                                m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND)));
        }

        public SequentialCommandGroup buildRemoveAlgaeCommand(ElevatorLevel startLevel, ElevatorLevel endLevel) {
                return new SequentialCommandGroup(
                                new PrintCommand("Running remove algae"),
                                new ParallelCommandGroup(
                                                //m_coral.wristExtendCommand(),
                                                new DriveOffset(m_swerve, m_Limelight, 0.7, 0.0),
                                                new GoToLevel(m_elevator, startLevel)),
                                new ParallelCommandGroup(
                                        m_coral.wristExtendCommand(),
                                        new DriveDistance2(m_swerve, () -> .24, 0).withTimeout(0.5)),
                                new GoToLevel(m_elevator, endLevel).withTimeout(.55),
                                new DriveDistance2(m_swerve, () -> .7, 180),
                                new StopDrive(m_swerve),
                                m_coral.wristRetractCommand(),
                                m_elevator.runOnce(() -> m_elevator.setLevel(ElevatorLevel.GROUND)),
                                new PrintCommand("Remove algae complete"));
        }
        
        public Drivetrain getDrivetrain() {
                return this.m_swerve;
        }

        private void configurePathPlanner() {
                //autoChooser.addOption("DriveForward", "DriveForward"); // Permanent choice
                autoChooser.addOption("OnePieceAuto", "OnePieceAuto"); // Permanent choice
                autoChooser.addOption("Left2Piece", "Left2Piece"); // Permanent choice
                autoChooser.addOption("Right2Piece", "Right2Piece"); // Permanent choice
                // For multi-step, create name to be name of multi-step, then have object be the
                // name of the first step
                // MultiStep example below
                // autoChooser.addOption("MultiStepRight", "Starting2Reef2"); // Permanent
                // choice
                // autoChooser.addOption("MultiStepLeft", "Starting7Reef4"); // Permanent choice
        }

        public SequentialCommandGroup buildTwoPieceAuto(String pathToReef, int tag1,
                        String pathToCoralStn, String pathCoralToReef, int tag2, String pathToCoralStn2,
                        double forwardDistM, boolean scoreLeft) {
                return new SequentialCommandGroup(
                                m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)),
                                new StopDrive(m_swerve),
                                getAutonomousCommand(pathToReef, true),
                                m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL4)),
                                buildScoreOffsetAutoCommand(scoreLeft),
                                new StationaryWait(m_swerve, .1),

                                getAutonomousCommand(pathToCoralStn, false),
                                new DriveDistance2(m_swerve, () -> .55, 180).withTimeout(.9),
                                new StopDrive(m_swerve),
                                new WaitUntilCommand(m_coral::detectsCoral).withTimeout(.4),

                                getAutonomousCommand(pathCoralToReef, false),
                                buildScoreOffsetCommand(scoreLeft),

                                getAutonomousCommand(pathToCoralStn2, false),
                                new DriveDistance2(m_swerve, () -> .55, 180).withTimeout(0.7),
                                new StopDrive(m_swerve),
                                m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)));
        }

        public SequentialCommandGroup buildTwoPieceAutoBumpered(String pathToReef, int tag1,
                        String pathToCoralStn, String pathCoralToReef, int tag2, double forwardDistM) {
                return new SequentialCommandGroup(
                                m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)),
                                getAutonomousCommand(pathToReef, true),
                                new StationaryWait(m_swerve, 0.5), // Testing purposes
                                buildScoreBumperedUpCommand(false, .25),
                                new StationaryWait(m_swerve, 0.5), // Testing purposes
                                getAutonomousCommand(pathToCoralStn, false),
                                new DriveDistance2(m_swerve, () -> .1, 180).withTimeout(0.3),
                                new StopDrive(m_swerve),
                                new StationaryWait(m_swerve, 1.0),
                                getAutonomousCommand(pathCoralToReef, false),
                                new StopDrive(m_swerve),
                                new StationaryWait(m_swerve, .5), // Testing purposes
                                buildScoreBumperedUpCommand(false, .25),
                                new StationaryWait(m_swerve, .5), // Testing purposes
                                m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)));
        }

        public void startAutonomous() {
                String auto = autoChooser.getSelected();
                SequentialCommandGroup start;
                Optional<Alliance> ally = DriverStation.getAlliance();
                if (auto.equals("Left2Piece")) { // For testing
                        int tag1 = 20;
                        int tag2 = 19;
                        if (ally.isPresent()) {
                                if (ally.get() == Alliance.Red) {
                                        tag1 = 11;
                                        tag2 = 6;
                                }
                        }
                        start = buildTwoPieceAuto("Starting2Reef2",
                                        tag1, "Reef2Player1",
                                        "Player1Reef1", tag2, "Reef1Player1", 0.16, false);
                        start.schedule();
                } else if (auto.equals("Right2Piece")) {
                        int tag1 = 22;
                        int tag2 = 17;
                        if (ally.isPresent()) {
                                if (ally.get() == Alliance.Red) {
                                        tag1 = 9;
                                        tag2 = 8;
                                }
                        }
                        start = buildTwoPieceAuto("Starting6Reef4",
                                        tag1, "Reef4Player2",
                                        "Player2Reef5", tag2, "Reef5Player2", 0.16, true);
                        start.schedule();
                } else if (auto.equals("OnePieceAuto")) {
                        start = new SequentialCommandGroup(
                                        m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)),
                                        m_elevator.runOnce(() -> m_elevator.setLevelFlag(ElevatorLevel.LEVEL4)),
                                        getAutonomousCommand("OnePieceAuto", true),
                                        new StationaryWait(m_swerve, .2),
                                        buildScoreOffsetAutoCommand(false),
                                        buildSelectRemoveAlgaeCommand(),
                                        m_swerve.runOnce(() -> m_swerve.setEnableVisionPoseInputs(false)),
                                        new StopDrive(m_swerve));

                        start.schedule();
                } else {
                        System.err.println("Invalid auto routine specified");
                }
        }

        public Command getAutonomousCommand(String pathName, boolean resetOdometry) {
                List<Waypoint> waypoints;
                PathPlannerPath path;
                Waypoint first;
                System.out.println("getAutoCommand building auto for " + pathName);
                try {
                        path = PathPlannerPath.fromPathFile(pathName);
                        if (resetOdometry) {
                                Optional<Pose2d> pose = path.getStartingHolonomicPose();
                                Optional<Alliance> ally = DriverStation.getAlliance();
                                waypoints = path.getWaypoints();
                                first = waypoints.get(0);
                                if (ally.isPresent()) {
                                        if (ally.get() == Alliance.Red) {
                                                System.out.println("Flipping start location for red");
                                                first = first.flip();
                                        }
                                }
                                if (pose.isPresent()) {
                                        m_swerve.resetStartingTranslation(first.anchor());
                                        System.out.println(first.toString());
                                } else {
                                        System.out.println("Error getting PathPlanner pose");
                                }
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

        public void reinitialize() {
                this.m_Limelight.setLimelightPipeline(LimelightConstants.defaultPipeline);
                this.m_elevator.updateConstants();
                this.m_elevator.resetEncoder();
                this.m_coral.reinit();
                this.setPIDConstants();
        }

        public ElevatorSubsystem getElevator() {
                return m_elevator;
        }

        public void setPIDConstants() {
                // Configure the drive train tuning constants from the dashboard
                for (SwerveModule m : m_swerve.getSwerveModules()) {
                        m.getDrivePidController().setP(m_driveP.getDouble(Constants.DrivetrainConstants.drivePID[0]));
                        m.getDriveFeedForward().setKs(
                                        m_driveFFStatic.getDouble(Constants.DrivetrainConstants.driveFeedForward[0]));
                        m.getDriveFeedForward().setKv(
                                        m_driveFFVel.getDouble(Constants.DrivetrainConstants.driveFeedForward[1]));
                        m.getDriveFeedForward().setKa(m_driveAccel.getDouble(0.0));
                        m.getTurnPidController().setP(m_turnP.getDouble(Constants.DrivetrainConstants.turnPID[0]));
                        m.getTurnPidController().setI(m_turnI.getDouble(DrivetrainConstants.turnPID[1]));
                        m.getTurnFeedForward().setKs(m_turnFFStatic.getDouble(DrivetrainConstants.turnFeedForward[0]));
                        m.getTurnFeedForward().setKv(m_turnFFVel.getDouble(DrivetrainConstants.turnFeedForward[1]));
                }
        }

        public boolean shouldRemoveAlgae(){
                ElevatorLevel elevatorState = m_elevator.getLevel();
                if (elevatorState == ElevatorLevel.LEVEL4){
                        if (Controller.kDriveController.getHID().getRightBumperButton() || Controller.kDriveController.getHID().getLeftBumperButton())
                                return true;
                }
                return false;
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
