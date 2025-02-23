package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.io.OutputStream;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.MotorPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorSubsystem extends SubsystemBase {
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

    private final NetworkTable m_table;
    private final StringPublisher m_table_level;
    private final DoublePublisher m_encoder;
    protected final DoublePublisher desiredVelocityPub;
    private final BooleanPublisher m_bottomlimitSwitch;
    private final BooleanPublisher m_toplimitSwitch;
    private final SparkFlex m_motorLeader;
    private final SparkFlex m_motorFollower;
    private final DoublePublisher outputVoltagePreClampPub;
    private final DoublePublisher outputVoltagePostClampPub;

    protected final MotorPublisher m_motorPublisherLeader;
    protected final MotorPublisher m_motorPublisherFollower;

    private final ElevatorFeedforward m_feedforward;
    private final ProfiledPIDController m_controller;

    protected DigitalInput bottomLimitSwitch;
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

    protected double outputVoltage = 0;
    protected double m_lastSpeed = 0.0;
    protected double m_lastTime = 0.0;
    protected boolean m_testMode = false;

    public ElevatorSubsystem(NetworkTableInstance nt) {
        m_table = nt.getTable(getName());
        m_table_level = m_table.getStringTopic("level").publish();
        m_encoder = m_table.getDoubleTopic("Encoder Position").publish();
        desiredVelocityPub = m_table.getDoubleTopic("Desired Vel").publish();
        m_bottomlimitSwitch = m_table.getBooleanTopic("Bottom Limit Switch").publish();
        m_toplimitSwitch = m_table.getBooleanTopic("Top Limit Switch").publish();
        outputVoltagePreClampPub = m_table.getDoubleTopic("Output V pre lamp").publish();
        outputVoltagePostClampPub = m_table.getDoubleTopic("Output V post lamp").publish();

        m_elevatorKs = m_table.getDoubleTopic("Ks").getEntry(0.0);
        m_elevatorKg = m_table.getDoubleTopic("Kg").getEntry(0.0);
        m_elevatorKv = m_table.getDoubleTopic("Kv").getEntry(0.0);
        m_elevatorKa = m_table.getDoubleTopic("Ka").getEntry(0.0);
        m_elevatorKp = m_table.getDoubleTopic("Kp").getEntry(0.0);
        m_elevatorKi = m_table.getDoubleTopic("Ki").getEntry(0.0);
        m_elevatorKd = m_table.getDoubleTopic("Kd").getEntry(0.0);
        m_elevatorKMaxVel = m_table.getDoubleTopic("KMaxVel").getEntry(0.0);
        m_elevatorKMaxAccel = m_table.getDoubleTopic("KMaxAccel").getEntry(0.0);
        
        m_elevatorKs.set(0.0);
        m_elevatorKg.set(0.0);
        m_elevatorKv.set(0.0);
        m_elevatorKa.set(0.0);
        m_elevatorKp.set(0.0);
        m_elevatorKi.set(0.0);
        m_elevatorKd.set(0.0);
        m_elevatorKMaxVel.set(0.0);
        m_elevatorKMaxAccel.set(0.0);

        m_motorLeader = new SparkFlex(Constants.MechID.kElevatorFrontCanId, MotorType.kBrushless);
        m_motorFollower = new SparkFlex(Constants.MechID.kElevatorBackCanId, MotorType.kBrushless);

        bottomLimitSwitch = new DigitalInput(Constants.Elevator.kBottomLimitSwitch);
        topLimitSwitch = new DigitalInput(Constants.Elevator.kTopLimitSwitch);

        SparkBaseConfig motorConf = new SparkFlexConfig();
        motorConf.smartCurrentLimit(40);
        motorConf.idleMode(IdleMode.kBrake);
        motorConf.inverted(false);
        m_motorLeader.configure(motorConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkBaseConfig followerConf = new SparkFlexConfig();
        followerConf.smartCurrentLimit(40);
        followerConf.idleMode(IdleMode.kBrake);
        followerConf.inverted(false);
        followerConf.follow(m_motorLeader, false);
        m_motorFollower.configure(followerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_motorLeader.setVoltage(0);
        m_motorLeader.getEncoder().setPosition(0);
        m_motorFollower.getEncoder().setPosition(0);
        
        m_motorPublisherLeader = new MotorPublisher(m_motorLeader, m_table, "leader");
        m_motorPublisherFollower = new MotorPublisher(m_motorFollower, m_table, "follower");

        m_feedforward = new ElevatorFeedforward(
                Constants.Elevator.FF.kS,
                Constants.Elevator.FF.kG,
                Constants.Elevator.FF.kV,
                Constants.Elevator.FF.kA);

        m_controller = new ProfiledPIDController(
                Constants.Elevator.PID.kP,
                Constants.Elevator.PID.kI,
                Constants.Elevator.PID.kD,
                new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration));

        setLevel(ElevatorLevel.GROUND);
    }

    public void setLevel(ElevatorLevel level) {
        m_level = level;
        m_table_level.set(level.toString());
        
        m_controller.setGoal(this.m_level.toHeight());
    }

    public void levelUp() {
        int currentLevel = ElevatorLevel.toInt(m_level);
        if (currentLevel < ElevatorLevel.toInt(ElevatorLevel.LEVEL4))
        {
            setLevel(ElevatorLevel.fromInt(currentLevel+1));
        }
    }

    public void levelDown() {
        int currentLevel = ElevatorLevel.toInt(m_level);
        if (currentLevel > ElevatorLevel.toInt(ElevatorLevel.GROUND))
        {
            setLevel(ElevatorLevel.fromInt(currentLevel-1));
        }
    }

    public ElevatorFeedforward getFeedforward(){
        return m_feedforward;
    }

    public ProfiledPIDController getPID(){
        return m_controller;
    }

    public ElevatorLevel getLevel() {
        return m_level;
    }

    public void updateConstants(){
        m_feedforward.setKs(m_elevatorKs.get());
        m_feedforward.setKg(m_elevatorKg.get());
        m_feedforward.setKv(m_elevatorKv.get());
        m_feedforward.setKa(m_elevatorKa.get());
        m_controller.setP(m_elevatorKp.get());
        m_controller.setI(m_elevatorKi.get());
        m_controller.setD(m_elevatorKd.get());
        m_controller.setConstraints(new Constraints(m_elevatorKMaxVel.get(), m_elevatorKMaxAccel.get()));
    }

    public void setVoltageTest(double voltage) {
        System.out.println("Setting Elevator voltage " + voltage);
        double currentPosition = m_motorLeader.getEncoder().getPosition();
        outputVoltage = voltage;
        if (!bottomLimitSwitch.get()){
            m_motorLeader.getEncoder().setPosition(0);
            if (outputVoltage < 0){
                outputVoltage = 0;
                
            }   
        }
        if (topLimitSwitch.get()){
            if (outputVoltage > 0){
                outputVoltage = 0;
            }   
        }

        if (currentPosition > 14.0 && outputVoltage > 0.0)
        {
            System.out.println("Cut off output due to max height");
            outputVoltage = 0.0;
        }
        m_motorLeader.setVoltage(outputVoltage);
    }

    public void setTestMode(boolean testMode) {
        this.m_testMode = testMode;
    }
 
    @Override
    public void periodic() {
        m_motorPublisherLeader.update();
        m_motorPublisherFollower.update();

        if (!this.m_testMode) {
            double currentPosition = m_motorLeader.getEncoder().getPosition();
            double targetVelocity = m_controller.getSetpoint().velocity;
            double targetAcceleration = (targetVelocity - this.m_lastSpeed)
                    / (Timer.getFPGATimestamp() - this.m_lastTime);

            double actualVelocity = m_motorLeader.getEncoder().getVelocity()/60.;

            double pidVal = m_controller.calculate(currentPosition, this.m_level.toHeight());
            double FFVal = m_feedforward.calculate(m_controller.getSetpoint().velocity);

            outputVoltage = (pidVal + FFVal);
            this.m_lastSpeed = actualVelocity;
            this.m_lastTime = Timer.getFPGATimestamp();

            if (!bottomLimitSwitch.get()){
                m_motorLeader.getEncoder().setPosition(0);
                if (outputVoltage < 0){
                    outputVoltage = 0;
                    
                }   
            }
            if (topLimitSwitch.get()){
                if (outputVoltage > 0){
                    outputVoltage = 0;
                }   
            }

            if (currentPosition > 14.0 && outputVoltage > 0.0)
            {
                System.out.println("Cut off output due to max height");
                outputVoltage = 0.0;
            }

            outputVoltagePreClampPub.set(outputVoltage);
            outputVoltage = MathUtil.clamp(outputVoltage, -6.0, 6.0);
            outputVoltagePostClampPub.set(outputVoltage);
            m_motorLeader.setVoltage(outputVoltage);
            desiredVelocityPub.set(targetVelocity);
        }
        m_encoder.set(m_motorLeader.getEncoder().getPosition());
        m_bottomlimitSwitch.set(!bottomLimitSwitch.get());
        m_toplimitSwitch.set(topLimitSwitch.get()); 
    }
}
