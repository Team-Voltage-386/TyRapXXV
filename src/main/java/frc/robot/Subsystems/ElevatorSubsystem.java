package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.logging.Level;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.MotorPublisher;
import frc.robot.Utils.TrapezoidController;
import frc.robot.Utils.SafeableSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class ElevatorSubsystem extends SafeableSubsystem {
    public enum ElevatorLevel {
        /**
         * The ground level of the elevator, where the human player can load coral.
         */
        GROUND,

        /**
         * AKA the trough level
         */
        LEVEL1,
        LEVEL2,
        LEVEL3,
        LEVEL4,
        LEVEL5,
        LEVEL6,
        LEVEL7;

        /**
         * Converts an integer to an ElevatorLevel
         * 
         * @param i
         * @return the ElevatorLevel corresponding to the integer, or GROUND if the
         *         integer is not recognized
         */
        public static ElevatorLevel fromInt(int i) {
            switch (i) {
                case 0:
                    return GROUND;
                case 1:
                    return LEVEL1;
                case 2:
                    return LEVEL2;
                case 3:
                    return LEVEL3;
                case 4:
                    return LEVEL4;
                case 5:
                    return LEVEL5;
                case 6:
                    return LEVEL6;
                case 7:
                    return LEVEL7;
                default:
                    return GROUND;
            }
        }

        /**
         * Converts an ElevatorLevel to an integer
         * 
         * @param level
         * @return the integer corresponding to the ElevatorLevel
         */
        public static int toInt(ElevatorLevel level) {
            switch (level) {
                case GROUND:
                    return 0;
                case LEVEL1:
                    return 1;
                case LEVEL2:
                    return 2;
                case LEVEL3:
                    return 3;
                case LEVEL4:
                    return 4;
                case LEVEL5:
                    return 5;
                case LEVEL6:
                    return 6;
                case LEVEL7:
                    return 7;
                default:
                    return 0;
            }
        }

        /**
         * Converts an ElevatorLevel to its height in encoder-specific
         * 'rotations'/'ticks'
         * 
         * @return the height of the ElevatorLevel in encoder-specific
         *         'rotations'/'ticks'
         */
        public double toHeight() {
            switch (this) {
                case GROUND:
                    return ground;
                case LEVEL1:
                    return level1;
                case LEVEL2:
                    return level2;
                case LEVEL3:
                    return level3;
                case LEVEL4:
                    return level4;
                case LEVEL5:
                    return level5;
                case LEVEL6:
                    return level6;
                case LEVEL7:
                    return level7;
                default:
                    return ground;
            }
        }

        public void setDashboard(double height) {
            switch (this) {
                case GROUND:
                    ground = Constants.Elevator.Heights.kGround;
                    break;
                case LEVEL1:
                    level1 = height;
                    break;
                case LEVEL2:
                    level2 = height;
                    break;
                case LEVEL3:
                    level3 = height;
                    break;
                case LEVEL4:
                    level4 = height;
                    break;
                case LEVEL5:
                    level5 = height;
                    break;
                case LEVEL6:
                    level6 = height;
                    break;
                case LEVEL7:
                    level7 = height;
                    break;
                default:
                    ground = Constants.Elevator.Heights.kGround;
                    break;
            }
        }

        /**
         * Converts an ElevatorLevel to a human-readable string
         * 
         * @return the human-readable string corresponding to the ElevatorLevel
         */
        public String toString() {
            switch (this) {
                case LEVEL1:
                    return String.format("LEVEL1 (%.2f rotations)", level1);
                case LEVEL2:
                    return String.format("LEVEL2 (%.2f rotations)", level2);
                case LEVEL3:
                    return String.format("LEVEL3 (%.2f rotations)", level3);
                case LEVEL4:
                    return String.format("LEVEL4 (%.2f rotations)", level4);
                case LEVEL5:
                    return String.format("LEVEL5 (%.2f rotations)", level5);
                case LEVEL6:
                    return String.format("LEVEL6 (%.2f rotations)", level6);
                case LEVEL7:
                    return String.format("LEVEL6 (%.2f rotations)", level7);
                default:
                    return String.format("GROUND (%.2f rotations)", ground);
            }
        }

        double ground = Constants.Elevator.Heights.kGround;
        double level1 = Constants.Elevator.Heights.kLevel1;
        double level2 = Constants.Elevator.Heights.kLevel2;
        double level3 = Constants.Elevator.Heights.kLevel3;
        double level4 = Constants.Elevator.Heights.kLevel4;
        double level5 = Constants.Elevator.Heights.kLevel5;
        double level6 = Constants.Elevator.Heights.kLevel6;
        double level7 = Constants.Elevator.Heights.kLevel7;
    }

    private ElevatorLevel m_level = ElevatorLevel.GROUND;
    private ElevatorLevel m_levelFlag = ElevatorLevel.GROUND;
    protected boolean isAnyLevelSet = false;

    private final NetworkTable m_table;
    private final StringPublisher m_table_level;
    private final DoublePublisher m_encoder;
    protected final DoublePublisher desiredVelocityPub;
    protected final DoublePublisher desiredPositionPub;
    private final BooleanPublisher m_bottomlimitSwitch;
    private final BooleanPublisher m_toplimitSwitch;
    private final SparkFlex m_motorLeader;
    private final SparkFlex m_motorFollower;
    private final DoublePublisher outputVoltagePreClampPub;
    private final DoublePublisher outputVoltagePostClampPub;

    protected final MotorPublisher m_motorPublisherLeader;
    protected final MotorPublisher m_motorPublisherFollower;

    private final ElevatorFeedforward m_feedforward;
    private final PIDController m_pidController;
    protected TrapezoidController m_trapController;
    protected RelativeEncoder encoder;

    protected DigitalInput bottomLimitSwitch; // make sure limit switches are not unhooked or it freaks out
    protected DigitalInput topLimitSwitch;

    protected DoubleEntry m_elevatorKs;
    protected DoubleEntry m_elevatorKg;
    protected DoubleEntry m_elevatorKv;
    protected DoubleEntry m_elevatorKa;
    protected DoubleEntry m_elevatorKp;
    protected DoubleEntry m_elevatorKi;
    protected DoubleEntry m_elevatorKd;
    protected DoubleEntry m_elevatorKMaxVel;
    protected DoubleEntry m_elevatorKMaxAccel;
    protected DoubleEntry m_elevatorKProportion;
    protected DoubleEntry LEVEL_1;
    protected DoubleEntry LEVEL_2;
    protected DoubleEntry LEVEL_3;
    protected DoubleEntry LEVEL_4;
    protected DoubleEntry LEVEL_5;
    protected DoubleEntry LEVEL_6;
    protected DoubleEntry LEVEL_7;

    protected double outputVoltage = 0;
    protected double desiredPosition = 0;
    protected double currentPosition = 0;
    protected double currentVelocity = 0;
    protected double targetVelocity = 0;
    protected double m_lastSpeed = 0.0;
    protected double m_lastTime = 0.0;
    protected boolean m_testMode = false;
    protected boolean m_manualMode = false;
    protected double m_manualSpeed = 0.0;
    protected double m_lastDesiredPosition = 0.0;

    public ElevatorSubsystem(NetworkTableInstance nt) {
        m_table = nt.getTable(getName());
        m_table_level = m_table.getStringTopic("level").publish();
        m_encoder = m_table.getDoubleTopic("Encoder Position").publish();
        desiredVelocityPub = m_table.getDoubleTopic("Desired Vel").publish();
        desiredPositionPub = m_table.getDoubleTopic("Desired Pos").publish();
        m_bottomlimitSwitch = m_table.getBooleanTopic("Bottom Limit Switch").publish();
        m_toplimitSwitch = m_table.getBooleanTopic("Top Limit Switch").publish();
        outputVoltagePreClampPub = m_table.getDoubleTopic("Output V pre lamp").publish();
        outputVoltagePostClampPub = m_table.getDoubleTopic("Output V post lamp").publish();

        m_elevatorKs = m_table.getDoubleTopic("Ks").getEntry(Constants.Elevator.FF.kS);
        m_elevatorKg = m_table.getDoubleTopic("Kg").getEntry(Constants.Elevator.FF.kG);
        m_elevatorKv = m_table.getDoubleTopic("Kv").getEntry(Constants.Elevator.FF.kV);
        m_elevatorKa = m_table.getDoubleTopic("Ka").getEntry(Constants.Elevator.FF.kA);
        m_elevatorKp = m_table.getDoubleTopic("Kp").getEntry(Constants.Elevator.PID.kP);
        m_elevatorKi = m_table.getDoubleTopic("Ki").getEntry(Constants.Elevator.PID.kI);
        m_elevatorKd = m_table.getDoubleTopic("Kd").getEntry(Constants.Elevator.PID.kD);
        m_elevatorKMaxVel = m_table.getDoubleTopic("KMaxVel").getEntry(Constants.Elevator.kMaxVelocity);
        m_elevatorKMaxAccel = m_table.getDoubleTopic("KMaxAccel").getEntry(Constants.Elevator.kMaxAcceleration);
        m_elevatorKProportion = m_table.getDoubleTopic("KProp").getEntry(Constants.Elevator.kDecelProp);
        LEVEL_1 = m_table.getDoubleTopic("Level 1").getEntry(ElevatorLevel.LEVEL1.toHeight());
        LEVEL_2 = m_table.getDoubleTopic("Level 2").getEntry(ElevatorLevel.LEVEL2.toHeight());
        LEVEL_3 = m_table.getDoubleTopic("Level 3").getEntry(ElevatorLevel.LEVEL3.toHeight());
        LEVEL_4 = m_table.getDoubleTopic("Level 4").getEntry(ElevatorLevel.LEVEL4.toHeight());
        LEVEL_5 = m_table.getDoubleTopic("Level 5").getEntry(ElevatorLevel.LEVEL5.toHeight());
        LEVEL_6 = m_table.getDoubleTopic("Level 6").getEntry(ElevatorLevel.LEVEL6.toHeight());
        LEVEL_7 = m_table.getDoubleTopic("Level 7").getEntry(ElevatorLevel.LEVEL7.toHeight());

        m_elevatorKs.set(Constants.Elevator.FF.kS);
        m_elevatorKg.set(Constants.Elevator.FF.kG);
        m_elevatorKv.set(Constants.Elevator.FF.kV);
        m_elevatorKa.set(Constants.Elevator.FF.kA);
        m_elevatorKp.set(Constants.Elevator.PID.kP);
        m_elevatorKi.set(Constants.Elevator.PID.kI);
        m_elevatorKd.set(Constants.Elevator.PID.kD);
        m_elevatorKMaxVel.set(Constants.Elevator.kMaxVelocity);
        m_elevatorKMaxAccel.set(Constants.Elevator.kMaxAcceleration);
        m_elevatorKProportion.set(Constants.Elevator.kDecelProp);
        LEVEL_1.set(ElevatorLevel.LEVEL1.toHeight());
        LEVEL_2.set(ElevatorLevel.LEVEL2.toHeight());
        LEVEL_3.set(ElevatorLevel.LEVEL3.toHeight());
        LEVEL_4.set(ElevatorLevel.LEVEL4.toHeight());
        LEVEL_5.set(ElevatorLevel.LEVEL5.toHeight());
        LEVEL_6.set(ElevatorLevel.LEVEL6.toHeight());
        LEVEL_7.set(ElevatorLevel.LEVEL7.toHeight());

        m_motorLeader = new SparkFlex(Constants.MechID.kElevatorFrontCanId, MotorType.kBrushless);
        m_motorFollower = new SparkFlex(Constants.MechID.kElevatorBackCanId, MotorType.kBrushless);

        bottomLimitSwitch = new DigitalInput(Constants.Elevator.kBottomLimitSwitch);
        topLimitSwitch = new DigitalInput(Constants.Elevator.kTopLimitSwitch);

        SparkBaseConfig motorConf = new SparkFlexConfig();
        motorConf.smartCurrentLimit(60);
        motorConf.idleMode(IdleMode.kBrake);
        motorConf.inverted(true);
        m_motorLeader.configure(motorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkBaseConfig followerConf = new SparkFlexConfig();
        followerConf.smartCurrentLimit(60);
        followerConf.idleMode(IdleMode.kBrake);
        followerConf.inverted(false);
        followerConf.follow(m_motorLeader, false);
        m_motorFollower.configure(followerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_motorLeader.setVoltage(0);
        m_motorLeader.getEncoder().setPosition(0);
        m_motorFollower.getEncoder().setPosition(0);

        encoder = m_motorLeader.getEncoder();
        encoder.setPosition(0);

        m_motorPublisherLeader = new MotorPublisher(m_motorLeader, m_table, "leader");
        m_motorPublisherFollower = new MotorPublisher(m_motorFollower, m_table, "follower");

        m_feedforward = new ElevatorFeedforward(
                Constants.Elevator.FF.kS,
                Constants.Elevator.FF.kG,
                Constants.Elevator.FF.kV,
                Constants.Elevator.FF.kA);

        m_pidController = new PIDController(
                Constants.Elevator.PID.kP,
                Constants.Elevator.PID.kI,
                Constants.Elevator.PID.kD);

        m_trapController = new TrapezoidController(0.0, 0.05, .2, Constants.Elevator.kMaxVelocity,
                Constants.Elevator.kMaxAcceleration, 50.0,
                Constants.Elevator.kDecelProp);

        setLevel(ElevatorLevel.GROUND);
    }

    public void setLevel(ElevatorLevel level) {
        System.out.println("Calling set level to " + level);
        m_level = level;
        m_table_level.set(level.toString());
        desiredPosition = this.m_level.toHeight();
    }

    public void setLevelFlag(ElevatorLevel level) {
        m_levelFlag = level;
        isAnyLevelSet = true;
        System.out.println("Setting elevator level flag to: " + m_levelFlag);
    }

    public void setLevelUsingFlag() {
        // m_level = m_levelFlag;
        // m_table_level.set(m_level.toString());
        this.setLevel(m_levelFlag);
        System.out.println("Moving elevator using the flag to: " + m_level);
    }

    public ElevatorLevel getLevelFlag() {
        return m_levelFlag;
    }

    public boolean isValidAlgaeLevel() {
        if (m_levelFlag == ElevatorLevel.LEVEL1 || m_levelFlag == ElevatorLevel.LEVEL3) {
            return true;
        }
        return false;
    }

    public boolean isAnyLevelSet() {
        return isAnyLevelSet;
    }

    public void levelUp() {
        int currentLevel = ElevatorLevel.toInt(m_level);
        if (currentLevel < ElevatorLevel.toInt(ElevatorLevel.LEVEL4)) { //It was LEVEL4 before
            setLevel(ElevatorLevel.fromInt(currentLevel + 1));
        }
    }

    public void levelDown() {
        int currentLevel = ElevatorLevel.toInt(m_level);
        if (currentLevel > ElevatorLevel.toInt(ElevatorLevel.GROUND)) {
            setLevel(ElevatorLevel.fromInt(currentLevel - 1));
        }
    }

    public double getDesiredPosition() {
        return this.desiredPosition;
    }

    public ElevatorFeedforward getFeedforward() {
        return m_feedforward;
    }

    public PIDController getPID() {
        return m_pidController;
    }

    public ElevatorLevel getLevel() {
        return m_level;
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public void holdCurrentPosition() {
        desiredPosition = currentPosition;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public void resetEncoder() {
        if (isAtBottom()) {
            encoder.setPosition(0);
            setLevel(ElevatorLevel.GROUND);
        } else {
            System.err.println("Tried to reset elevator encoder when not at ground level");
        }
    }

    public void updateConstants() {
        m_feedforward.setKs(m_elevatorKs.get());
        m_feedforward.setKg(m_elevatorKg.get());
        m_feedforward.setKv(m_elevatorKv.get());
        m_feedforward.setKa(m_elevatorKa.get());
        m_pidController.setP(m_elevatorKp.get());
        m_pidController.setI(m_elevatorKi.get());
        m_pidController.setD(m_elevatorKd.get());
        m_trapController.setMaxVel(m_elevatorKMaxVel.get());
        m_trapController.setMaxAccel(m_elevatorKMaxAccel.get());
        m_trapController.setDecelKp(m_elevatorKProportion.get());
        ElevatorLevel.LEVEL1.setDashboard(LEVEL_1.get());
        ElevatorLevel.LEVEL2.setDashboard(LEVEL_2.get());
        ElevatorLevel.LEVEL3.setDashboard(LEVEL_3.get());
        ElevatorLevel.LEVEL4.setDashboard(LEVEL_4.get());
        ElevatorLevel.LEVEL5.setDashboard(LEVEL_5.get());
        ElevatorLevel.LEVEL6.setDashboard(LEVEL_6.get());
        ElevatorLevel.LEVEL7.setDashboard(LEVEL_7.get());
    }

    public void setVoltageTest(double voltage) {
        outputVoltage = voltage;
        if (voltage > 0.0) {
            targetVelocity = 0.1;
        } else if (voltage < 0.0) {
            targetVelocity = -0.1;
        } else {
            targetVelocity = 0.0;
        }
        handleLimits();
        m_motorLeader.setVoltage(outputVoltage);
    }

    public void manualUp() {
        m_manualMode = true;
        m_manualSpeed = Constants.Elevator.kManualSpeed;
    }

    public void manualDown() {
        m_manualMode = true;
        m_manualSpeed = -Constants.Elevator.kManualSpeed;
    }

    public void stopManualMode() {
        m_manualMode = false;
    }

    public void setTestMode(boolean testMode) {
        this.m_testMode = testMode;
    }

    public boolean isAtBottom() { // bottom limit states are swapped
        return (!bottomLimitSwitch.get());
    }

    public boolean isAtTop() {
        return topLimitSwitch.get();
    }

    public void handleLimits() {
        if (isAtBottom()) {
            if (outputVoltage < 0 || targetVelocity <= 0.0) {
                outputVoltage = 0;
            }
        }
        if (isAtTop()) {
            if (outputVoltage > 0) {
                outputVoltage = 0;
            }
        }
        if (currentPosition > Constants.Elevator.kElevatorMaxPos && outputVoltage > 0.0) {
            System.out.println("Cut off output due to max height");
            outputVoltage = 0.4;
        }
    }

    public void makeSafe() {
        setLevel(ElevatorLevel.GROUND);
    }

    @Override
    public void periodic() {
        m_motorPublisherLeader.update();
        m_motorPublisherFollower.update();

        if (!this.m_testMode) {
            currentPosition = encoder.getPosition();
            double currentDelta = desiredPosition - currentPosition;
            currentVelocity = encoder.getVelocity() / 60;
            targetVelocity = m_trapController.calculate(currentDelta, currentVelocity);

            if (desiredPosition != this.m_lastDesiredPosition) {
                if (Math.abs(currentVelocity) < 0.001) {
                    System.out.println("Reinit trap controller");
                    m_trapController.reinit(currentVelocity);
                }
            }

            if (m_manualMode) {
                if (currentPosition >= ElevatorLevel.LEVEL4.toHeight()) {
                    m_level = ElevatorLevel.LEVEL4;
                } else if (currentPosition >= ElevatorLevel.LEVEL6.toHeight()) {
                        m_level = ElevatorLevel.LEVEL6; //Level6 is between level3 & level4
                } else if (currentPosition >= ElevatorLevel.LEVEL3.toHeight()) {
                    m_level = ElevatorLevel.LEVEL3;
                } else if (currentPosition >= ElevatorLevel.LEVEL5.toHeight()) {
                    m_level = ElevatorLevel.LEVEL5; //Level5 is between level2 & level3
                } else if (currentPosition >= ElevatorLevel.LEVEL2.toHeight()) {
                    m_level = ElevatorLevel.LEVEL2;
                } else if (currentPosition >= ElevatorLevel.LEVEL1.toHeight()) {
                    m_level = ElevatorLevel.LEVEL1;
                } else {
                    m_level = ElevatorLevel.GROUND;
                }
                m_table_level.set(m_level.toString());
                desiredPosition = currentPosition;
                targetVelocity = m_manualSpeed;
            }
            // Enforce a velocity limit for safety until tuning complete
            targetVelocity = MathUtil.clamp(targetVelocity, -2.5, 12);

            double targetAcceleration = (targetVelocity - this.m_lastSpeed);

            double pidVal = m_pidController.calculate(currentPosition, desiredPosition);
            double FFVal = m_feedforward.calculate(targetVelocity, targetAcceleration);

            outputVoltage = pidVal + FFVal;
            this.m_lastSpeed = currentVelocity;
            this.m_lastTime = Timer.getFPGATimestamp();

            handleLimits();

            this.m_lastDesiredPosition = desiredPosition;
            outputVoltagePreClampPub.set(outputVoltage);
            if (currentPosition < 3) {
                outputVoltage = MathUtil.clamp(outputVoltage, -1.0, 6.0);
            } else if (currentPosition > Constants.Elevator.kElevatorMaxPos - 1.5) {
                outputVoltage = MathUtil.clamp(outputVoltage, -2.0, 1.0);
            } else {
                outputVoltage = MathUtil.clamp(outputVoltage, -4.0, 9.0);
            }
            outputVoltagePostClampPub.set(outputVoltage);
            desiredVelocityPub.set(targetVelocity);
            desiredPositionPub.set(desiredPosition);

            m_motorLeader.setVoltage(outputVoltage);
        }
        m_encoder.set(encoder.getPosition());
        m_bottomlimitSwitch.set(isAtBottom());
        m_toplimitSwitch.set(isAtTop());
    }
}
