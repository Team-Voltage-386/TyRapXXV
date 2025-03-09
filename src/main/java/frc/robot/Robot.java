// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {
    private RobotContainer m_container;

    @Override
    public void robotInit() {
        m_container = new RobotContainer();
        FollowPathCommand.warmupCommand().schedule();
        m_container.getDrivetrain().resetGyro();
        // Starts recording to data log
        DataLogManager.start();
        // Record both DS control and joystick data
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
        m_container.getDrivetrain().setFieldRelative(true);
        m_container.clearDefaultCommand();
        m_container.setTeleDefaultCommand();
        m_container.reinitialize();
        m_container.configureBindings();
        m_container.turnRumbleOff();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
        m_container.turnRumbleOff();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_container.reportTelemetry();
    }

    @Override
    public void disabledPeriodic() {
        // Uncomment these lines in order to output the swerve turn encoder values (to obtain offsets)
        SmartDashboard.putNumber("BackLeft", m_container.getDrivetrain().getBackLeftSwerveModule().getRawTurningPositionRadians());
        SmartDashboard.putNumber("BackRight", m_container.getDrivetrain().getBackRightSwerveModule().getRawTurningPositionRadians());
        SmartDashboard.putNumber("FrontLeft", m_container.getDrivetrain().getFrontLeftSwerveModule().getRawTurningPositionRadians());
        SmartDashboard.putNumber("FrontRight", m_container.getDrivetrain().getFrontRightSwerveModule().getRawTurningPositionRadians());
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
