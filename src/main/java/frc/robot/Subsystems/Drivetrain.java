// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Vector;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;

public class Drivetrain extends SubsystemBase {

    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // per second

    protected final Translation2d m_frontLeftLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXForward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYLeft);
    protected final Translation2d m_frontRightLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXForward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYRight);
    protected final Translation2d m_backLeftLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXBackward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYLeft);
    protected final Translation2d m_backRightLocation = new Translation2d(
            DrivetrainConstants.kDistanceMiddleToFrontMotor * DrivetrainConstants.kXBackward,
            DrivetrainConstants.kDistanceMiddleToSideMotor * DrivetrainConstants.kYRight);

    protected final SwerveModule m_frontLeft = new SwerveModule("FrontLeft",
            ID.kFrontLeftDrive,
            ID.kFrontLeftTurn,
            ID.kFrontLeftCANCoder,
            Offsets.kFrontLeftOffset,
            DrivetrainConstants.turnPID,
            DrivetrainConstants.drivePID,
            DrivetrainConstants.turnFeedForward,
            DrivetrainConstants.driveFeedForward,
            DrivetrainConstants.sparkFlex,
            true // Pass inverstion value
    );
    protected final SwerveModule m_frontRight = new SwerveModule("FrontRight",
            ID.kFrontRightDrive,
            ID.kFrontRightTurn,
            ID.kFrontRightCANCoder,
            Offsets.kFrontRightOffset,
            DrivetrainConstants.turnPID,
            DrivetrainConstants.drivePID,
            DrivetrainConstants.turnFeedForward,
            DrivetrainConstants.driveFeedForward,
            DrivetrainConstants.sparkFlex,
            false);
    protected final SwerveModule m_backLeft = new SwerveModule("BackLeft",
            ID.kBackLeftDrive,
            ID.kBackLeftTurn,
            ID.kBackLeftCANCoder,
            Offsets.kBackLeftOffset,
            DrivetrainConstants.turnPID,
            DrivetrainConstants.drivePID,
            DrivetrainConstants.turnFeedForward,
            DrivetrainConstants.driveFeedForward,
            DrivetrainConstants.sparkFlex,
            false);
    protected final SwerveModule m_backRight = new SwerveModule("BackRight",
            ID.kBackRightDrive,
            ID.kBackRightTurn,
            ID.kBackRightCANCoder,
            Offsets.kBackRightOffset,
            DrivetrainConstants.turnPID,
            DrivetrainConstants.drivePID,
            DrivetrainConstants.turnFeedForward,
            DrivetrainConstants.driveFeedForward,
            DrivetrainConstants.sparkFlex,
            true);
    protected final Vector<SwerveModule> m_swerveModules = new Vector<SwerveModule>(4);

    protected final Pigeon2 m_gyro;

    protected boolean fieldRelative = true;
    protected final ShuffleboardTab m_driveTab = Shuffleboard.getTab("drive subsystem");
    protected final SimpleWidget m_fieldRelativeWidget = m_driveTab.add("drive field relative", fieldRelative);
    protected final GenericEntry m_driveCommandedRotationSpeed = m_driveTab.add("drive commanded rotation", 0)
            .getEntry();

    /**
     * The order that you initialize these is important! Later uses of functions
     * like toSwerveModuleStates will return the same order that these are provided.
     * See
     * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    protected final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    protected final SwerveDrivePoseEstimator m_odometry;
    protected int counter = 0;
    protected boolean enableVisionPoseInputs;

    protected ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
    protected ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds();

    protected ExecutorService executorService = Executors.newFixedThreadPool(4);

    protected final Field2d field = new Field2d();

    public Drivetrain(Pigeon2 gyro) {
        // Zero at beginning of match. Zero = whatever direction the robot (more
        // specifically the gyro) is facing
        this.m_gyro = gyro;
        m_driveTab.add("field", field);

        m_swerveModules.add(m_frontLeft);
        m_swerveModules.add(m_frontRight);
        m_swerveModules.add(m_backLeft);
        m_swerveModules.add(m_backRight);

        m_odometry = new SwerveDrivePoseEstimator(
                m_kinematics,
                getGyroYawRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                },
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

        // Load the RobotConfig from the PathPlanner GUI settings
        RobotConfig ppConfig;
        try {
            ppConfig = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getRoboPose2d, // Robot pose supplier
                    this::resetOdo, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    this::driveChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    new PPHolonomicDriveController( // HolonomicPathFollowerConfig
                            new PIDConstants(DrivetrainConstants.drivePID[0], // Translation PID constants
                                    DrivetrainConstants.drivePID[1],
                                    DrivetrainConstants.drivePID[2]),
                            new PIDConstants(DrivetrainConstants.turnPID[0], // Rotation PID constants
                                    DrivetrainConstants.turnPID[1],
                                    DrivetrainConstants.turnPID[2])),
                    ppConfig,
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE            
                        return getAlliance() == DriverStation.Alliance.Red;
                    },
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }
    
    public void resetStartingPose(Pose2d newPose) {
        m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(), newPose);
    }

    public void resetStartingTranslation(Translation2d newTranslation) {
        m_odometry.resetTranslation(newTranslation);
    }

    public Pigeon2 getGyro() {
        return m_gyro;
    }

    public Alliance getAlliance() {
        return DriverStation.getAlliance().get();
    }

    /**
     * Resets Orientation of the robot
     */
    public void resetGyro() {
        if (getAlliance() == Alliance.Blue) {
            System.out.println("Initializing gyro to 180 for BLUE");
            m_gyro.setYaw(180.0);
        } else {
            System.out.println("Initializing gyro to 0 for RED");
            m_gyro.setYaw(0);
        }
    }

    public double getExpectedStartGyro() {
        if (getAlliance() == DriverStation.Alliance.Blue) {
            return 180.0;
        } else {
            return 0.0;
        }
    }

    public SwerveModule getBackLeftSwerveModule() {
        return m_backLeft;
    }

    public SwerveModule getBackRightSwerveModule() {
        return m_backRight;
    }

    public SwerveModule getFrontLeftSwerveModule() {
        return m_frontLeft;
    }

    public SwerveModule getFrontRightSwerveModule() {
        return m_frontRight;
    }

    public Vector<SwerveModule> getSwerveModules() {
        return m_swerveModules;
    }

    /**
     * Resets robot position on the field
     */
    public void resetOdo() {
        System.out.println("Calling resetOdo with no arguments");
        if (DriverStation.getRawAllianceStation().equals(AllianceStationID.Blue1)
                || DriverStation.getRawAllianceStation().equals(AllianceStationID.Blue2)
                || DriverStation.getRawAllianceStation().equals(AllianceStationID.Blue3)) {
            m_odometry.resetPosition(new Rotation2d(Math.PI), getModulePositions(),
                    new Pose2d(new Translation2d(getRoboPose2d().getX(), getRoboPose2d().getY()),
                            new Rotation2d(Math.PI)));
        } else {
            m_odometry.resetPosition(new Rotation2d(), getModulePositions(),
                    new Pose2d(new Translation2d(getRoboPose2d().getX(), getRoboPose2d().getY()),
                            new Rotation2d()));
        }
    }

    /**
     * Resets Odometry using a specific Pose2d
     * 
     * @param pose
     */
    public void resetOdo(Pose2d pose) {
        if (pose != null) {
            System.out.println("Calling resetOdo with pose " + pose);
            m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(), pose);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public boolean getFieldRelative() {
        return fieldRelative;
    }

    public void setFieldRelative(boolean isFieldRelative) {
        fieldRelative = isFieldRelative;
        m_fieldRelativeWidget.getEntry().setBoolean(fieldRelative);
    }

    public Command toggleFieldRelativeCommand() {
        return runOnce(() -> {
            this.setFieldRelative(!this.fieldRelative);
        });
    }

    /**
     * Module positions in the form of SwerveModulePositions (Module orientation and
     * the distance the wheel has travelled across the ground)
     * 
     * @return SwerveModulePosition[]
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
        };
    }

    /**
     * Get the yaw of gyro in Rotation2d form
     * 
     * @return chasis angle in Rotation2d
     */
    public Rotation2d getGyroYawRotation2d() {
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
    }

    public Rotation2d getPlayerStationRelativeYaw2d() {
        Rotation2d rot = Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
        if (getAlliance() == DriverStation.Alliance.Red) {
            rot = rot.rotateBy(Rotation2d.k180deg);
        }
        return rot;
    }

    public ChassisSpeeds getCommandeChassisSpeeds() {
        return commandedChassisSpeeds;
    }

    protected double driveMultiplier = 1;

    public void setDriveMult(double mult) {
        driveMultiplier = mult;
    }

    public Command setDriveMultCommand(double mult) {
        return Commands.runOnce(() -> setDriveMult(mult));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rotSpeed      Angular rate of the robot.
     * 
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed) {
        xSpeed = xSpeed * driveMultiplier;
        ySpeed = ySpeed * driveMultiplier;
        rotSpeed = rotSpeed * driveMultiplier;

        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                    getPlayerStationRelativeYaw2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        driveChassisSpeeds(chassisSpeeds);

        SmartDashboard.putNumber("desired X speed", xSpeed);
        SmartDashboard.putNumber("desired Y speed", ySpeed);
        SmartDashboard.putNumber("dsired rot speed", rotSpeed);
    }

    public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        m_driveCommandedRotationSpeed.setDouble(Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));
        commandedChassisSpeeds = chassisSpeeds;
    }

    public Pose2d getRoboPose2d() {
        return m_odometry.getEstimatedPosition();
    }

    public void setEnableVisionPoseInputs(boolean enableVisionPoseInputs) {
        this.enableVisionPoseInputs = enableVisionPoseInputs;
    }

    public void stopDriving() {
        commandedChassisSpeeds = new ChassisSpeeds();
        // Get velocity and position from each module
        SwerveModuleState frontLeftState = m_frontLeft.getState();
        SwerveModuleState frontRightState = m_frontRight.getState();
        SwerveModuleState backLeftState = m_backLeft.getState();
        SwerveModuleState backRightState = m_backRight.getState();
        // Command each module to zero speed while maintaining its current
        // angle
        frontLeftState.speedMetersPerSecond = 0.0;
        m_frontLeft.setDesiredState(frontLeftState);
        frontRightState.speedMetersPerSecond = 0.0;
        m_frontRight.setDesiredState(frontRightState);
        backLeftState.speedMetersPerSecond = 0.0;
        m_backLeft.setDesiredState(backLeftState);
        backRightState.speedMetersPerSecond = 0.0;
        m_backRight.setDesiredState(backRightState);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        Rotation2d rotationYaw = getGyroYawRotation2d();
        m_odometry.update(
                rotationYaw,
                getModulePositions());
        counter++;
        if (counter % 10 == 0 && enableVisionPoseInputs) {
            LimelightHelpers.SetRobotOrientation(ID.kFrontLimelightName, rotationYaw.getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ID.kFrontLimelightName);
            boolean doRejectUpdate = false;
            if(Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()) > 360) // if our angular velocity is too large, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if(mt2 != null) {
                if(mt2.tagCount == 0)
                {
                    doRejectUpdate = true;
                }
                if(!doRejectUpdate)
                {
                    m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                    m_odometry.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
                }
            }
        }

        // getting velocity vectors from each module
        SwerveModuleState frontLeftState = m_frontLeft.getState();
        SwerveModuleState frontRightState = m_frontRight.getState();
        SwerveModuleState backLeftState = m_backLeft.getState();
        SwerveModuleState backRightState = m_backRight.getState();

        // Converting module speeds to chassis speeds
        m_chassisSpeeds = m_kinematics.toChassisSpeeds(
                frontLeftState, frontRightState, backLeftState, backRightState);

        field.setRobotPose(getRoboPose2d());
    }

    @Override
    public void periodic() {
        updateOdometry();

        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(commandedChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        CountDownLatch latch = new CountDownLatch(4);
        executorService.execute(() -> {
            m_frontLeft.setDesiredState(swerveModuleStates[0]);
            latch.countDown();
        });
        executorService.execute(() -> {
            m_frontRight.setDesiredState(swerveModuleStates[1]);
            latch.countDown();
        });
        executorService.execute(() -> {
            m_backLeft.setDesiredState(swerveModuleStates[2]);
            latch.countDown();
        });
        executorService.execute(() -> {
            m_backRight.setDesiredState(swerveModuleStates[3]);
            latch.countDown();
        });
        try {
            latch.await();
        } catch (InterruptedException e) {
            // Pass
        }
    }
}
