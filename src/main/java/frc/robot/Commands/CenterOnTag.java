package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
//import frc.robot.TyRap24Constants.*;
import frc.robot.SparkJrConstants.*;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;

public class CenterOnTag extends Command {
    Drivetrain dt;
    Limelight ll;
    private static GenericEntry rotProportion = Shuffleboard.getTab("Limelight").add("rotProportion", 2).getEntry();
    private static GenericEntry xProportion = Shuffleboard.getTab("Limelight").add("xProportion", 2).getEntry();
    private static GenericEntry ySpeedEntry = Shuffleboard.getTab("Limelight").add("ySpeedEntry", 0.5).getEntry();
    private static GenericEntry xDisEntry = Shuffleboard.getTab("Limelight").add("xDisEntry", 0.5).getEntry();
    private static GenericEntry rotSpeedEntry = Shuffleboard.getTab("Limelight").add("rotSpeedEntry", 0.5).getEntry();
    private static GenericEntry yawAngleEntry = Shuffleboard.getTab("Limelight").add("yawAngleEntry", 0.5).getEntry();
    private static GenericEntry minVelEntry = Shuffleboard.getTab("Limelight")
            .add("minVelEntry", LimelightConstants.minLinearVelocity).getEntry();
    private static GenericEntry minAngVelEntry = Shuffleboard.getTab("Limelight")
            .add("minAngVelEntry", LimelightConstants.minAngVelocityDPS).getEntry();
    private static GenericEntry maxAngVelEntry = Shuffleboard.getTab("Limelight")
            .add("maxAngVelEntry", LimelightConstants.maxAngVelocityDPS).getEntry();

    public CenterOnTag(Drivetrain dt, Limelight ll) {

        this.dt = dt;
        this.ll = ll;
        addRequirements(dt);
    }

    private double xSpeed = 0;
    private double ySpeed = 0;
    private double rotSpeed = 0;
    private double minVel = 0;
    private double minAngVel = 0;
    private double maxAngVel = 0;
    private double currentRotProportion = 0.0;
    private double currentXProportion = 0.0;

    @Override
    public void initialize() {
        dt.setFieldRelative(false);
        minVel = minVelEntry.getDouble(LimelightConstants.minLinearVelocity);
        minAngVel = minAngVelEntry.getDouble(LimelightConstants.minAngVelocityDPS);
        maxAngVel = maxAngVelEntry.getDouble(LimelightConstants.maxAngVelocityDPS);
        currentRotProportion = rotProportion.getDouble(2);
        currentXProportion = xProportion.getDouble(1.0);
        System.out.println("minVel=" + minVel + " minAngVel=" + minAngVel + "  maxAngVel=" + maxAngVel);
    }

    @Override
    public void execute() {
        double rotAngleDegrees = -1 * ll.getFilteredYawDegrees();
        double xDis = -1 * ll.getxDistanceMeters();
        double desiredVel = Math.abs(xDis * currentXProportion);
        double desiredAngVel = Math.abs(rotAngleDegrees * currentRotProportion);
        if (Math.abs(ll.getxDistanceMeters()) < LimelightConstants.xDisThreshold) {
            ySpeed = 0;
        } else {
            ySpeed = Math.copySign(MathUtil.clamp(desiredVel, minVel, 1.5), xDis);
        }
        if (Math.abs(ll.getYawAngleDegrees()) < LimelightConstants.rotThreshold) {
            rotSpeed = 0;
        } else {
            rotSpeed = Math.copySign(MathUtil.clamp(desiredAngVel, minAngVel, maxAngVel), rotAngleDegrees);
        }
        ySpeedEntry.setDouble(ySpeed);
        rotSpeedEntry.setDouble(rotSpeed);
        xDisEntry.setDouble(xDis);
        yawAngleEntry.setDouble(rotAngleDegrees);
        dt.drive(xSpeed, ySpeed, Math.toRadians(rotSpeed));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(ll.getxDistanceMeters()) < LimelightConstants.xDisThreshold
                && Math.abs(ll.getYawAngleDegrees()) < LimelightConstants.rotThreshold) {
            dt.setFieldRelative(true);
            dt.drive(0, 0, 0);
            System.out.println("COT command complete");
            return true;
        }
        return false;
    }
}
