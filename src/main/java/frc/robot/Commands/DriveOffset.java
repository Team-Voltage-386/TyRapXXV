package frc.robot.Commands;

// Imports
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;
import frc.robot.Utils.CoordinateUtilities;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Utils.TrapezoidController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.*;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

// This Command combines DriveDistance and CenterOnTag, using Limelight and Odometry 
//      to drive to an april tag and then drive a specific offset left or right from it.
public class DriveOffset extends Command {
    // Create Drivetrain and Limelight objects
    Drivetrain dt;
    Limelight ll;

    // Shuffleboard for testing
    // Some values are taken from Constants
    private static GenericEntry rangeMEntry = Shuffleboard.getTab("DriveOffset").add("rangeM", 0).getEntry();
    private static GenericEntry bearingEntry = Shuffleboard.getTab("DriveOffset").add("bearing", 0).getEntry();
    private static GenericEntry currentXVelEntry = Shuffleboard.getTab("DriveOffset").add("currentXVel", 0)
            .getEntry();
    private static GenericEntry currentYVelEntry = Shuffleboard.getTab("DriveOffset").add("currentYVel", 0)
            .getEntry();
    private static GenericEntry minVelEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetMinVelEntry", LimelightConstants.driveOffsetMinVel).getEntry();
    private static GenericEntry maxVelEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetMaxVelEntry", LimelightConstants.driveOffsetMaxVel).getEntry();
    private static GenericEntry maxAccEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetMaxAccEntry", LimelightConstants.driveOffsetMaxAccMSS).getEntry();
    private static GenericEntry maxDccEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetMaxDccEntry", LimelightConstants.driveOffsetMaxDccMSS).getEntry();
    protected static GenericEntry xOffsetEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetXOffset", LimelightConstants.driveOffsetXOffset).getEntry();
    protected static GenericEntry yOffsetEntry = Shuffleboard.getTab("DriveOffset")
            .add("DriveOffsetYOffset", LimelightConstants.driveOffsetYOffset).getEntry();
    protected static GenericEntry omegaDps = Shuffleboard.getTab("DriveOffset")
            .add("OmegaSpeedDps", 0).getEntry();
    protected static GenericEntry angleErrorEntry = Shuffleboard.getTab("DriveOffset").add("DriveOffsetAngleError", 0)
            .getEntry();
    private static GenericEntry currentLinearSpeedEntry = Shuffleboard.getTab("DriveOffset")
            .add("CurrentLinearSpeed", 0)
            .getEntry();
    private static GenericEntry decelKpEntry = Shuffleboard.getTab("DriveOffset")
            .add("DecelKp", LimelightConstants.driveOffsetKp)
            .getEntry();

    // Variables created for DriveOffset
    private boolean isLeft;
    private int id = 999999999;
    private double xOffset;
    private double yOffset;
    private Pose2d tagPose;
    private Pose2d currentPose;
    private Pose2d desiredPose;
    protected Transform2d cameraToRobot;

    // Variables from CenterOnTag
    private double currentVel;
    private ChassisSpeeds calcVel;
    private ChassisSpeeds chassisSpeed;
    private double chassisMagnitude;
    private double minVel;
    private double rangeM;
    private double bearingDeg;
    private double angleError;
    protected double commandedSpeed;
    protected final double threshold = LimelightConstants.driveOffsetRangeMThreshold;
    private int counter;
    protected TrapezoidController trapezoidController;
    protected boolean useDashboardEntries;
    protected double constructedXOffsetM;
    protected double constructedYOffsetM;

    // Constructors
    public DriveOffset(Drivetrain dt, Limelight ll, boolean isLeft) {
            this.dt = dt;
            this.ll = ll;
            this.isLeft = isLeft;
            this.useDashboardEntries = true;
            this.id = 999999999;
            addRequirements(dt);
            // 2D transform between robot and camera frames
            // Currently get offset from SparkJrConstants, but can change later
            cameraToRobot = new Transform2d(-1 * Offsets.cameraOffsetForwardM, 0, new Rotation2d());
    }
    
    public DriveOffset(Drivetrain dt, Limelight ll, boolean isLeft, int id) {
            this.dt = dt;
            this.ll = ll;
            this.isLeft = isLeft;
            this.id = id;
            this.useDashboardEntries = true;
            addRequirements(dt);
            // 2D transform between robot and camera frames
            // Currently get offset from SparkJrConstants, but can change later
            cameraToRobot = new Transform2d(-1 * Offsets.cameraOffsetForwardM, 0, new Rotation2d());
    }

        public DriveOffset(Drivetrain dt, Limelight ll, double xOffsetM, double yOffsetM) {
                this.dt = dt;
                this.ll = ll;
                this.isLeft = false;
                this.useDashboardEntries = false;
                this.constructedXOffsetM = xOffsetM;
                this.constructedYOffsetM = yOffsetM;
                addRequirements(dt);
                // 2D transform between robot and camera frames
                // Currently get offset from SparkJrConstants, but can change later
                cameraToRobot = new Transform2d(-1 * Offsets.cameraOffsetForwardM, 0, new Rotation2d());
                System.out.println("Running drive offset with set x and y offsets");
        }

    @Override
    public void initialize() {
        // Set id
        if (id == 999999999) {
                LimelightHelpers.SetFiducialIDFiltersOverride(ID.kFrontLimelightName, new int[] {});
        }
        else {
                LimelightHelpers.SetFiducialIDFiltersOverride(ID.kFrontLimelightName, new int[] { id });
                System.out.println("Set ID");
        }

        // Create a new Trapezoid profile
        if (useDashboardEntries) {
                xOffset = xOffsetEntry.getDouble(0.3);
                yOffset = yOffsetEntry.getDouble(0.0);
                // Change offset based on direction, so we can go to left or right reef goal
                if (isLeft) {
                        // Left is negative Y
                        yOffset *= -1;
                }
        } else {
                xOffset = constructedXOffsetM;
                yOffset = constructedYOffsetM;
        }
        // Get minimun velocity from Constants and shuffleboard
        minVel = minVelEntry.getDouble(LimelightConstants.driveOffsetMinVel);
        desiredPose = getDesiredPose();

        // Reset counter
        counter = 0;

        trapezoidController = new TrapezoidController(0.0, threshold,minVel,
            maxVelEntry.getDouble(LimelightConstants.driveOffsetMaxVel),
            maxAccEntry.getDouble(LimelightConstants.driveOffsetMaxAccMSS),
            maxDccEntry.getDouble(LimelightConstants.driveOffsetMaxDccMSS),
            decelKpEntry.getDouble(1.0));
    }

    // This function uses limelight and odometry to report the desired pose relative to the field
    public Pose2d getDesiredPose() {
        // Get values from limelight
        double rotAngleDegrees = -1 * ll.getFilteredYawDegrees();
        double yDis = -1 * ll.getxDistanceMeters();
        double xDis = ll.getzDistanceMeters();
        // Get april tag position from camera
        tagPose = new Pose2d(xDis, yDis, new Rotation2d(Math.toRadians(rotAngleDegrees + 180.0)));
        // Calculate desired offset using shuffleboard and at 0 degrees
        Transform2d desiredOffset = new Transform2d(xOffset, yOffset, new Rotation2d());
        // Calculate robot-relative desired pose
        Pose2d desiredPoseRobotRelative = tagPose.plus(desiredOffset).plus(cameraToRobot);
        // Get current robot pose from odometry
        currentPose = dt.getRoboPose2d();
        // Get desired position from odometry
        Pose2d desiredPoseField = currentPose
                .plus(new Transform2d(desiredPoseRobotRelative.getX(), desiredPoseRobotRelative.getY(),
                        new Rotation2d(Math.toRadians(rotAngleDegrees))));
        // Print outs for testing
        //System.out.println("currentPose = " + currentPose);
        //System.out.println("tagPose = " + tagPose);
        //System.out.println("desiredPoseRobotRelative = " + desiredPoseRobotRelative);
        //System.out.println("desiredPoseField = " + desiredPoseField);

        return desiredPoseField;
    }

    @Override
    public void execute() {
        // Update desired pose every so often
        if (ll.getTimeSinceValid() == 0 && (counter % 5 == 0)) {
            desiredPose = getDesiredPose();
        }
        counter++;
        // Get current pose
        currentPose = dt.getRoboPose2d();
        // Calculate distance to the new position from the current one
        rangeM = CoordinateUtilities.distanceTo(currentPose, desiredPose);
        // Calculate bearing (desired angle) in degrees
        bearingDeg = CoordinateUtilities.robotBearingTo(currentPose, desiredPose);
        // Get chassis speed
        chassisSpeed = dt.getChassisSpeeds();
        // Get chassis magnitude
        chassisMagnitude = CoordinateUtilities.getChassisMagnitude(chassisSpeed);
     
        double commandedVel = trapezoidController.calculate(rangeM, chassisMagnitude);

        // Calculate x and y components of desired velocity
        calcVel = CoordinateUtilities.courseSpeedToLinearVelocity(bearingDeg, commandedVel);
        
        // Calculate total time left
        //var currentVel = Math.max(commandedSpeed, minVel);
        double remainingTime = 1.0; // Placeholder. Calculate based on remaining distance
        
        // Calculate angle error
        angleError = desiredPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
        if (angleError > Math.PI) {
                angleError -= 2*Math.PI;
        }
        if (Math.abs(angleError) > LimelightConstants.driveOffsetAngleError) {
                if (remainingTime > 0) {
                        // Calculate angular speed
                        calcVel.omegaRadiansPerSecond = angleError / remainingTime;
                        calcVel.omegaRadiansPerSecond = Math.copySign(Math.max(Math.abs(calcVel.omegaRadiansPerSecond), Math.toRadians(10.0)), calcVel.omegaRadiansPerSecond);
                } else {
                        // Give robot a chance to turn to desired angle even after we've reached the correct distance
                        calcVel.omegaRadiansPerSecond = Math.toRadians(
                                Math.copySign(Math.toRadians(8.0), angleError));
                }
        }
        omegaDps.setDouble(Math.toDegrees(calcVel.omegaRadiansPerSecond));
        // Drive
        dt.driveChassisSpeeds(calcVel);
        // Set range, bearing, velocities, and angleError in Shuffleboard
        rangeMEntry.setDouble(rangeM);
        bearingEntry.setDouble(bearingDeg);
        currentXVelEntry.setDouble(calcVel.vxMetersPerSecond);
        currentYVelEntry.setDouble(calcVel.vyMetersPerSecond);
        angleErrorEntry.setDouble(Math.toDegrees(angleError));
        currentLinearSpeedEntry.setDouble(chassisMagnitude);
    }

    @Override
    public boolean isFinished() {
        if (rangeM <= threshold && Math.abs(angleError) <= LimelightConstants.driveOffsetAngleError) {
                LimelightHelpers.SetFiducialIDFiltersOverride(ID.kFrontLimelightName, Constants.ID.allAprilIDs);
                System.out.println("Reset IDs");
            return true;
        }
        return false;
    }
}
