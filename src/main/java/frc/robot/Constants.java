package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Constants {
    public static class ID {
        // Swerve motor controller IDs
        public static final int kFrontLeftTurn = 1;
        public static final int kFrontLeftDrive = 2;

        public static final int kFrontRightTurn = 3;
        public static final int kFrontRightDrive = 4;

        public static final int kBackLeftTurn = 5;
        public static final int kBackLeftDrive = 6;

        public static final int kBackRightTurn = 7;
        public static final int kBackRightDrive = 8;

        // Swerve CanCoder IDs
        public static final int kFrontRightCANCoder = 21;
        public static final int kBackRightCANCoder = 24;
        public static final int kBackLeftCANCoder = 23;
        public static final int kFrontLeftCANCoder = 22;

        // Pigeon
        public static final int kGyro = 2;

        // Limelight
        public static final String kFrontLimelightName = "limelight-c";
        public static final int[] allAprilIDs = new int[] { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
                18, 19, 20, 21, 22 };
        public static final int[] reefAprilIDs = new int[] { 6, 7, 8, 9, 10, 11, 17,
                18, 19, 20, 21, 22 };
        public static final int[] algaeAprilIDs = new int[] { 3, 4, 5, 14, 15, 16 };
        public static final int[] playerAprilIDs = new int[] { 1, 2, 12, 13 };
    }

    public static class Climber {
        // TODO
        public static final int kLowerSolenoidCANID1 = 4;
        public static final int kLowerSolenoidCANID2 = 5;

        public static final int kClampSolenoidCANID1 = 2;
        public static final int kClampSolenoidCANID2 = 3;

        public static final int kRampSolenoidCANID1 = 6;
        public static final int kRampSolenoidCANID2 = 7;

        public static final double kClimbMotorVoltage = 12;
        public static final double kMaxEncoderPos = 0.43;
        public static final double kMinEncoderPos = 0.28;
    }

    public static class Coral {
        public static final double kWristMotorVoltage = 6.0;
        public static final double kWristMotorVoltageReverse = 10.0;
        // down -4.4
        // up +6
        public static final double kCoralEjectVoltage = .65;
        public static final double kCoralEjectVoltageLevel4 = .75;
        public static final double kMaxEncoderPos = .255;
        public static final double kMinEncoderPos = .062;
        //public static final double kLimitEncoderPos = .037; //HARD STOP
        public static final int wristCounterLimit = 5;
        public static final int kWristMaxRolloverCount = 1;
        public static final double kWristPropRetracting = 60.0;
        public static final double kWristPropExtending = 50.0;
        public static final double kWristMinPosition = -.375;
        public static final double kWristMaxPosition = 0.0;
        public static final double kWristRelativeExtension = -0.37;
        public static final double kWristPositionTolerance = 0.01; // Rotations
        //public static final double kWristManualSpeedRotationsPerSec = 0.25;
    }

    public static class MechID {
        public static final int kAlgaeMotorCanId = 41;
        public static final int kClimberCanId = 10;
        // Elevator
        public static final int kElevatorBackCanId = 11;
        public static final int kElevatorFrontCanId = 12;
        // Coral Manipulator
        public static final int kCoralWristCanId = 13;
        public static final int kCoralWheelCanId = 14;

    }

    public static class Deadbands {
        public static final double kLeftJoystickDeadband = 0.06;
        public static final double kRightJoyStickDeadband = 0.06;
    }

    public static class DrivetrainConstants {
        // Distance in meters
        public static final double kDistanceMiddleToFrontMotor = 0.339852;
        public static final double kDistanceMiddleToSideMotor = 0.289052;
        public static final double kDriveBaseRadius = Math.sqrt( // distance from the middle to the furthest wheel
                kDistanceMiddleToFrontMotor * kDistanceMiddleToFrontMotor +
                        kDistanceMiddleToSideMotor * kDistanceMiddleToSideMotor);

        public static final int kXForward = 1;
        public static final int kXBackward = -1;
        public static final int kYLeft = 1;
        public static final int kYRight = -1;

        // ITS TUNED. NO TOUCH!
        public static final double[] turnPID = { 4.5, 1.0, 0.0 };
        public static final double[] drivePID = { 2, 0.0, 0.0 };
        public static final double[] turnFeedForward = { 0.3, 0.2 };
        public static final double[] driveFeedForward = { 0.17, 2.255 };

        public static final boolean kInvertTurn = true;
        public static final double kMaxPossibleSpeed = 5.3; // meters per second

        public static final boolean sparkFlex = true;
    };

    public static final class Modules {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    }

    public static class Controller {
        public static final int kDriveControllerID = 0;
        public static final int kManipControllerID = 1;

        /**
         * Rate limiters make joystick inputs more gentle; 1/3 sec from 0 to 1.
         */
        public static final double kRateLimitXSpeed = 150.0;
        public static final double kRateLimitYSpeed = 150.0;
        public static final double kRateLimitRot = 70.0;
        public static final double kMaxNecessarySpeed = DrivetrainConstants.kMaxPossibleSpeed * 0.8;

        public static final CommandXboxController kDriveController = new CommandXboxController(kDriveControllerID);
        public static final CommandXboxController kManipulatorController = new CommandXboxController(
                kManipControllerID);
    }

    public static class Offsets {
        // Ensure that the encoder offsets are between -Pi & Pi
        /**
         * Encoder offsets. These can be obtained by straightening
         * all wheels in the forward position and reading the encoder values
         * from the dashboard by uncommenting the lines in Robot.java
         * disabledPeriodic.
         */
        public static final double kFrontLeftOffset = 0.8268;
        public static final double kFrontRightOffset = 0.158;
        public static final double kBackLeftOffset = 2.269;
        public static final double kBackRightOffset = 1.101;

        // Camera Positioning
        public static final double cameraOffsetForwardM = 0.08;
        public static final double cameraOffsetFromFrontBumber = 0.38;
    }

    public static class LimelightConstants {
        public static final int defaultPipeline = 2;
        // For CenterOnTag
        public static final double minXVelocity = 0.1;
        public static final double maxXVelocity = 1.0;
        public static final double minYVelocity = 0.1;
        public static final double maxYVelocity = 1.0;
        public static final double minAngVelocityDPS = 0;
        public static final double maxAngVelocityDPS = 20;
        public static final double xDisThreshold = 0.03;
        public static final double rotThreshold = 1.0;
        public static final double azimuthFieldOfViewDeg = 29.0;
        public static final double xOffset = 0.0;
        public static final double yOffset = 0.7;
        public static final double maxAngAccMSS = 8;
        public static final double maxAngDccMSS = 16;
        // For ProportionalController & DriveDistance as well
        public static final double maxAccMSS = 3;
        public static final double maxDccMSS = 8;
        public static final double minVelocity = 0.1;
        public static final double maxVelocity = 5.0;
        public static final double offset = 0.0;
        public static final double proportion = 2;
        public static final double threshold = .02;
        public static final double driveDistanceProp = 3;
        // For DriveOffset
        public static final double driveOffsetXOffset = 0.7;
        public static final double driveOffsetYOffset = 0.17;
        public static final double driveOffsetMaxAccMSS = 2.5;
        public static final double driveOffsetMaxDccMSS = 8;
        public static final double driveOffsetMinVel = 0.1;
        public static final double driveOffsetMaxVel = 3.0;
        public static final double driveOffsetAngleError = 0.04;
        public static final double driveOffsetRangeMThreshold = 0.01;
        public static final double driveOffsetKp = 3.0;
    }

    public static final class SensorID {
        public static final int kIRSensorPort = 7;
    }

    public class Elevator {
        public static class Heights {
            public static final double kGround = -1.0;
            public static final double kLevel1 = 5.3; // 3.5
            public static final double kLevel2 = 7.6;
            public static final double kLevel5 = 8.0; //Level5 is between level2 & level3 (ALGAE ONLY)
            public static final double kLevel3 = 14.6;
            public static final double kLevel6 = 16.0; //Level6 is between level3 & level4 (ALGAE ONLY)
            public static final double kLevel7 = 21.9;
            public static final double kLevel4 = 25.15; 
        }

        public static class PID {
            public static final double kP = 0.0; // not tuned
            public static final double kI = 0.0; // not tuned
            public static final double kD = 0.0; // not tuned
        }

        public static class FF { // not tuned
            public static final double kS = 0.1; // static friction (V)
            public static final double kG = 0.65; // gravity (V)
            public static final double kV = 1.6; // volts per velocity (V/(m/s))
            public static final double kA = 0.0; // volts per acceleration (V/(m/s^2))
        }

        public static final double kMaxVelocity = 3.5;
        public static final double kMaxAcceleration = 3.0;
        public static final double kDecelProp = 0.5;

        public static final double kElevatorGearRatio = 0;
        public static final double kElevatorDrumRadius = 0;

        public static final double kManualSpeed = 0.5;
        public static final double kElevatorMaxPos = 27.65;

        public static final int kBottomLimitSwitch = 4;
        public static final int kTopLimitSwitch = 8;
    }

    public static class AlgaeGrabber {
        public static final double kMotorCurrentThreshold = 1.0;
        public static final double kIntakeSpeed = 0.90;
        public static final double kEjectSpeed = 0.55;

        public static final int kSolenoidCANID1 = 0;
        public static final int kSolenoidCANID2 = 1;

    }
}