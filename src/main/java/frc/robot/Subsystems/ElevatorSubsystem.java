package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorSubsystem extends SubsystemBase {
    public enum ElevatorLevel{ 
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
        LEVEL4;

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
                    return Constants.Elevator.Heights.kGround;
                case LEVEL1:
                    return Constants.Elevator.Heights.kLevel1;
                case LEVEL2:
                    return Constants.Elevator.Heights.kLevel2;
                case LEVEL3:
                    return Constants.Elevator.Heights.kLevel3;
                case LEVEL4:
                    return Constants.Elevator.Heights.kLevel4;
                default:
                    return Constants.Elevator.Heights.kGround;
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
                    return String.format("LEVEL1 (%.2f rotations)", Constants.Elevator.Heights.kLevel1);
                case LEVEL2:
                    return String.format("LEVEL2 (%.2f rotations)", Constants.Elevator.Heights.kLevel2);
                case LEVEL3:
                    return String.format("LEVEL3 (%.2f rotations)", Constants.Elevator.Heights.kLevel3);
                case LEVEL4:
                    return String.format("LEVEL4 (%.2f rotations)", Constants.Elevator.Heights.kLevel4);
                default:
                    return String.format("GROUND (%.2f rotations)", Constants.Elevator.Heights.kGround);
            }
        }
    }

    private ElevatorLevel m_level = ElevatorLevel.GROUND;
    private ElevatorLevel m_levelFlag = ElevatorLevel.GROUND;

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

        m_trapController = new TrapezoidController(0.0, 0.05, .1, Constants.Elevator.kMaxVelocity,
                Constants.Elevator.kMaxAcceleration, 7.5,
                Constants.Elevator.kDecelProp);

        setLevel(ElevatorLevel.GROUND);
    }

    public void setLevel(ElevatorLevel level) {
        m_level = level;
        m_table_level.set(level.toString());
        desiredPosition = this.m_level.toHeight();
    }

    public void setLevelFlag(ElevatorLevel level) {
        m_levelFlag = level;
        System.out.println("Setting elevator level flag to: " + m_levelFlag);
    }

    public void setLevelUsingFlag() {
        m_level = m_levelFlag;
        m_table_level.set(m_level.toString());
        System.out.println("Moving elevator using the flag to: " + m_level);
    }

    public void levelUp() {
        int currentLevel = ElevatorLevel.toInt(m_level);
        if (currentLevel < ElevatorLevel.toInt(ElevatorLevel.LEVEL4)) {
            setLevel(ElevatorLevel.fromInt(currentLevel + 1));
        }
    }

    public void levelDown() {
        int currentLevel = ElevatorLevel.toInt(m_level);
        if (currentLevel > ElevatorLevel.toInt(ElevatorLevel.GROUND)) {
            setLevel(ElevatorLevel.fromInt(currentLevel - 1));
        }
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

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public void resetEncoder() {
        encoder.setPosition(0);
        setLevel(ElevatorLevel.GROUND);
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
    }

    public void setVoltageTest(double voltage) {
        outputVoltage = voltage;
        if (voltage > 0.0) {
            targetVelocity = 0.1;
        } else if (voltage < 0.0)
        {
            targetVelocity = -0.1;
        } else 
        {
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
                desiredPosition = currentPosition;
                targetVelocity = m_manualSpeed;
            }
            // Enforce a velocity limit for safety until tuning complete
            targetVelocity = MathUtil.clamp(targetVelocity, -1, 1.2);

            double targetAcceleration = (targetVelocity - this.m_lastSpeed);

            double pidVal = m_pidController.calculate(currentPosition, desiredPosition);
            double FFVal = m_feedforward.calculate(targetVelocity, targetAcceleration);

            outputVoltage = pidVal + FFVal;
            this.m_lastSpeed = currentVelocity;
            this.m_lastTime = Timer.getFPGATimestamp();

            handleLimits();

            this.m_lastDesiredPosition = desiredPosition;
            outputVoltagePreClampPub.set(outputVoltage);
            if (currentPosition < 1.0) {
                outputVoltage = MathUtil.clamp(outputVoltage, -1.0, 6.0);
            } else if (currentPosition > Constants.Elevator.kElevatorMaxPos - 1.5) {
                outputVoltage = MathUtil.clamp(outputVoltage, -1.0, 1.0);
            }
            else {
                outputVoltage = MathUtil.clamp(outputVoltage, -1.0, 4.0);
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
