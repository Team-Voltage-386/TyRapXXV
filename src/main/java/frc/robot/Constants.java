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
    }
    public static class Climber {
        // TODO
        public static final int kLowerSolenoidCANID1 = 4;
        public static final int kLowerSolenoidCANID2 = 5;

        public static final int kClampSolenoidCANID1 = 2;
        public static final int kClampSolenoidCANID2 = 3;

        public static final double kClimbMotorVoltage = 1.0;
        public static final double kMaxEncoderPos = 0.5;
        public static final double kMinEncoderPos = 0.218;
    }

    public static class MechID {
        public static final int kAlgaeMotorCanId = 41;
        public static final int kClimberCanId = 10;
        //Elevator
        public static final int kElevatorBackCanId = 11;
        public static final int kElevatorFrontCanId = 12;
        //Coral Manipulator
        public static final int kCoralWristCanId = 30;
        public static final int kCoralWheelCanId = 31;
    
    }

    public static class Deadbands {
        public static final double kLeftJoystickDeadband = 0.06;
        public static final double kRightJoyStickDeadband = 0.04;
    }

    public static class DriveTrainConstants {
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
        public static final double[] turnPID = { 1.5, 0.2, 0.0 };
        public static final double[] drivePID = { 3, 0.1, 0.0 };
        public static final double[] turnFeedForward = { 0.0, 0.3 };
        public static final double[] driveFeedForward = { 0.0, 2.675 };

        public static final boolean kInvertTurn = true;
        public static final double kMaxPossibleSpeed = 3.5; // meters per second

        public static final boolean sparkFlex = true;
    };

    public static final class Modules {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    }

    public static class Controller {
        public static final int kDriveControllerID = 1;
        public static final int kManipControllerID = 0;

        /**
         * Rate limiters make joystick inputs more gentle; 1/3 sec from 0 to 1.
         */
        public static final double kRateLimitXSpeed = 100.0;
        public static final double kRateLimitYSpeed = 100.0;
        public static final double kRateLimitRot = 70.0;
        public static final double kMaxNecessarySpeed = DriveTrainConstants.kMaxPossibleSpeed * 0.8;

        public static final CommandXboxController kDriveController = new CommandXboxController(kDriveControllerID);
        public static final CommandXboxController kManipulatorController = new CommandXboxController(kManipControllerID);
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
        public static final double cameraOffsetForwardM = 0.10;
        public static final double cameraOffsetFromFrontBumber = 0.38;
    }

    public static class LimelightConstants {
        // For CenterOnTag
        public static final double minXVelocity = 0.1;
        public static final double maxXVelocity = 1.0;
        public static final double minYVelocity = 0.1;
        public static final double maxYVelocity = 1.0;
        public static final double minAngVelocityDPS = 0;
        public static final double maxAngVelocityDPS = 20;
        public static final double xDisThreshold = 0.03;
        public static final double yDisThreshold = 0.03;
        public static final double rotThreshold = 1.0;
        public static final double azimuthFieldOfViewDeg = 29.0;
        public static final double xOffset = 0.0;
        public static final double yOffset = 0.7;
        public static final double maxAngAccMSS = 8;
        public static final double maxAngDccMSS = 16;
        // For ProportionalController & DriveDistance as well
        public static final double maxAccMSS = 4;
        public static final double maxDccMSS = 3;
        public static final double minVelocity = 0.15;
        public static final double maxVelocity = 5.0;
        public static final double offset = 0.0;
        public static final double proportion = 2;
        public static final double threshold = .02;
        // For DriveOffset
        public static final double driveOffsetXOffset = 0.7;
        public static final double driveOffsetYOffset = 0.3;
        public static final double driveOffsetMaxAccMSS = 2;
        public static final double driveOffsetMaxDccMSS = 2;
        public static final double driveOffsetMinVel = 0.1;
        public static final double driveOffsetMaxVel = 1.5;
        public static final double driveOffsetAngleError = 0.015;
        public static final double driveOffsetRangeMThreshold = 0.02;
    }

    public class Elevator {
        public static class Heights {
            public static final double kGround = 0.0;
            public static final double kLevel1 = 2.0;
            public static final double kLevel2 = 4.0;
            public static final double kLevel3 = 6.0;
            public static final double kLevel4 = 8.0;
        }

        public static class PID {
            public static final double kP = 1.3; // not tuned
            public static final double kI = 0.0; // not tuned
            public static final double kD = 0.7; // not tuned
        }

        public static class FF {
            public static final double kS = 1.1; // static friction (V)
            public static final double kG = 1.2; // gravity (V)
            public static final double kV = 1.3; // volts per velocity (V/(m/s))
            public static final double kA = 0.0; // volts per acceleration (V/(m/s^2))
        }

        public static final double kMaxVelocity = 4.0;
        public static final double kMaxAcceleration = 4.0;

        public static final double kElevatorGearRatio = 0;
        public static final double kElevatorDrumRadius = 0;


        public static final int kBottomLimitSwitch = 9;
        public static final int kTopLimitSwitch = 8;
    }

    public static class AlgaeGrabber {
        public static final double kMotorCurrentThreshold = 1.0;
        public static final double kIntakeSpeed = 0.60;
        public static final double kEjectSpeed = 0.90;

        public static final int kSolenoidCANID1 = 0;
        public static final int kSolenoidCANID2 = 1;

    }
}
// public static class ElevatorConstants {
// //sim
// public static final double maxHeightM = 2.0;
// public static final double minHeightM = 0.8;
// public static final double maxVelocityMps = 2.0;

// public static final double kMaxVelocity = 4.0;
// public static final double kMaxAcceleration = 4.0;
// public static final double kMaxVoltage = 3;
// //elevator pid not tuned
// public static final double kp = 1.3;
// public static final double ki = 0.0;
// public static final double kd = 0.7;
// //elevator ff not tuned
// public static final double ks = 1.1; //static friction (V)
// public static final double kg = 1.2; //gravity (V)
// public static final double kv = 1.3; //volts per velocity (V/(m/s))
// public static final double ka = 0.0; //volts per acceleration (V/(m/s^2))
// //fake values
// public static final double kMamotor2xElevatorHeight = 1.75; //determine units
// public static final double kMinElevatorHeight = 0.0;
// public static double[] HEIGHT_STAGE = {0, 1, 2, 3};
// public static final double kElevatorGearing = 10;
// public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
// public static final double kCarriageMass = 4.0; // kg
// //demoboard IDs
// public static final int leadCANID = 13;
// public static final int followCANID = 14;
// }
