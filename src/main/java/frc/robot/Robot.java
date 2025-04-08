// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.StopDrive;

public class Robot extends TimedRobot {
    private RobotContainer m_container;
    protected boolean gyroCorrect;
    protected Alliance currentAlliance;

    @Override
    public void robotInit() {
        gyroCorrect = false;
        m_container = new RobotContainer();
        currentAlliance = m_container.getDrivetrain().getAlliance();
        
        FollowPathCommand.warmupCommand().schedule();
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    @Override
    public void autonomousInit() {
        m_container.getDrivetrain().setFieldRelative(true);
        m_container.clearDefaultCommand();
        m_container.setAutoDefaultCommand();
        m_container.reinitialize();
        m_container.startAutonomous();
    }

    @Override
    public void teleopInit() {
        StopDrive stop = new StopDrive(m_container.getDrivetrain());
        stop.schedule();
        m_container.getDrivetrain().setFieldRelative(true);
        m_container.clearDefaultCommand();
        m_container.setTeleDefaultCommand();
        m_container.reinitialize();
        m_container.configureBindings();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_container.reportTelemetry();
    }

    @Override
    public void disabledPeriodic() {
        if (currentAlliance != m_container.getDrivetrain().getAlliance()) {
            currentAlliance = m_container.getDrivetrain().getAlliance();
            System.out.println("Alliance changed to " + currentAlliance);
            gyroCorrect = false;
        }
        if (!gyroCorrect) {
            double expectedGyro = m_container.getDrivetrain().getExpectedStartGyro();
            double currentGyro = m_container.getDrivetrain().getGyroYawRotation2d().getDegrees();
            System.out.println("Expected Gyro: " + expectedGyro + " current: " + currentGyro);
            if (Math.abs(expectedGyro - currentGyro) > 1.0) {
                System.out.println("Gyro delta too large");
                m_container.getDrivetrain().resetGyro();
                System.out.println("Reset gyro from robot periodic");
            } else {
                gyroCorrect = true;
                m_container.getDrivetrain().resetOdo();
            }
        }
        // Uncomment these lines in order to output the swerve turn encoder values (to obtain offsets)
        //SmartDashboard.putNumber("BackLeft", m_container.getDrivetrain().getBackLeftSwerveModule().getRawTurningPositionRadians());
        //SmartDashboard.putNumber("BackRight", m_container.getDrivetrain().getBackRightSwerveModule().getRawTurningPositionRadians());
        //SmartDashboard.putNumber("FrontLeft", m_container.getDrivetrain().getFrontLeftSwerveModule().getRawTurningPositionRadians());
        //SmartDashboard.putNumber("FrontRight", m_container.getDrivetrain().getFrontRightSwerveModule().getRawTurningPositionRadians());
    }

    @Override
    public void testInit() {
       // m_container.getDrivetrain().setFieldRelative(true);
       //m_container.clearDefaultCommand();
        //m_container.setTeleDefaultCommand();
        m_container.reinitialize();
        m_container.configureTestBindings();
    }
    @Override
    public void testPeriodic() {
    }

}
