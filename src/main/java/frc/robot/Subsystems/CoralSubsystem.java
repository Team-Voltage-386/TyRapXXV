package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Commands.MoveCoralManipulator;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorLevel;
import frc.robot.Utils.MotorPublisher;
import frc.robot.Utils.SafeableSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*
 * 
 * different speeds up vs down 
 * IR sensor 
 * holde break logic
 */
public class CoralSubsystem extends SafeableSubsystem {
    private final NetworkTable m_table;

    private final SparkMax m_wristMotor;
    private final SparkMax m_coralGrabberMotor;
    private final ElevatorSubsystem el;

    private final MotorPublisher m_wristMotorPublisher;
    private final MotorPublisher m_coralGrabberMotorPublisher;

    protected final AbsoluteEncoder m_wristEncoder;
    private final DoublePublisher m_wristEncoderPub;
    protected final DoublePublisher m_wristPositionPub;

    private final DigitalInput m_irSensor;
    private final DoublePublisher m_irSensorPub;

    protected DoubleEntry kWristMotorVoltageForward;
    protected DoubleEntry kWristMotorVoltageReverse;
    protected DoubleEntry kWristMotorSpeedForward;
    protected DoubleEntry kWristMotorSpeedReverse;
    protected DoubleEntry kWristPropExtendingEntry;
    protected DoubleEntry kWristPropRetractingEntry;
    protected DoubleEntry kCoralEjectSpeedEntry;
    protected DoubleEntry kCoralEjectSpeedLevel4Entry;
    protected DoubleEntry minEncoderPositionEntry;
    protected DoubleEntry wristExtendedPositionEntry;

    private double counter;
    private double start;
    private boolean ejectActive = false;
    private Timer timer;

    private double lastWristEncoderVal;
    private boolean wristStopped = false;
    protected double minEncoderPosition = 0.0;
    protected double wristExtendedPosition = 0.0;
    private int wristCounter = 0;
    protected int wristRolloverCount = 0;
    protected double wristPosition = 0; // Absolute wrist position starting at 0 when retracted and 
                                        // accounting for encoder rollover
    protected double wristDesiredPosition = 0;
    protected PIDController wristPidController;

    public enum CoralState {
        WAITING, INTAKING, HOLDING, EJECTING;
    };

    protected CoralState state = CoralState.WAITING;
    protected boolean enabled = false;

    public CoralSubsystem(NetworkTableInstance nt, ElevatorSubsystem el) {
        this.el = el;
        m_table = nt.getTable(getName());

        m_coralGrabberMotor = new SparkMax(Constants.MechID.kCoralWheelCanId, MotorType.kBrushless);
        m_wristMotor = new SparkMax(Constants.MechID.kCoralWristCanId, MotorType.kBrushed);

        kWristMotorVoltageForward = m_table.getDoubleTopic("wrist motor voltage forward")
                .getEntry(Constants.Coral.kWristMotorVoltage);
        kWristMotorVoltageReverse = m_table.getDoubleTopic("wrist motor voltage reverse")
                .getEntry(Constants.Coral.kWristMotorVoltageReverse);
        kWristPropExtendingEntry = m_table.getDoubleTopic("WristExtendingKp")
                .getEntry(Constants.Coral.kWristPropExtending);
        kWristPropRetractingEntry = m_table.getDoubleTopic("WristRetractingKp")
                .getEntry(Constants.Coral.kWristPropRetracting);
        kCoralEjectSpeedEntry = m_table.getDoubleTopic("CoralEjectVoltage")
                .getEntry(Constants.Coral.kCoralEjectVoltage);
        kCoralEjectSpeedLevel4Entry = m_table.getDoubleTopic("CoralEjectVoltageL4")
                .getEntry(Constants.Coral.kCoralEjectVoltageLevel4);
        minEncoderPositionEntry = m_table.getDoubleTopic("MinEncoderPosition")
                .getEntry(0.0);
        wristExtendedPositionEntry = m_table.getDoubleTopic("WristExtendedPosition")
                .getEntry(Constants.Coral.kWristRelativeExtension);

        // kWristMotorSpeedForward = m_table.getDoubleTopic("wrist motor speed
        // forward").getEntry(0.0);
        // kWristMotorSpeedReverse = m_table.getDoubleTopic("wrist motor speed
        // reverse").getEntry(0.0);

        kWristMotorVoltageForward.set(Constants.Coral.kWristMotorVoltage);
        kWristMotorVoltageReverse.set(Constants.Coral.kWristMotorVoltageReverse);
        kWristPropExtendingEntry.set(Constants.Coral.kWristPropExtending);
        kWristPropRetractingEntry.set(Constants.Coral.kWristPropRetracting);
        kCoralEjectSpeedEntry.set(Constants.Coral.kCoralEjectVoltage);
        kCoralEjectSpeedLevel4Entry.set(Constants.Coral.kCoralEjectVoltageLevel4);
        // kWristMotorSpeedForward.set(0.0);
        // kWristMotorSpeedReverse.set(0.0);

        m_coralGrabberMotorPublisher = new MotorPublisher(m_coralGrabberMotor, m_table, "Grabber Motor");
        m_wristMotorPublisher = new MotorPublisher(m_wristMotor, m_table, "Wrist Motor");

        m_wristEncoder = m_coralGrabberMotor.getAbsoluteEncoder();
        m_wristEncoderPub = nt.getDoubleTopic("Wrist Encoder").publish();
        m_wristPositionPub = nt.getDoubleTopic("Wrist Position").publish();

        m_irSensor = new DigitalInput(Constants.SensorID.kIRSensorPort);
        m_irSensorPub = nt.getDoubleTopic("IR Sensor").publish();

        lastWristEncoderVal = m_wristEncoder.getPosition();
        wristPosition = 0.0;
        wristPidController = new PIDController(Constants.Coral.kWristPropExtending, 0, 0);

        timer = new Timer();
    }

    public void ejectCoral() {
        if (state == CoralState.HOLDING) {
            ejectActive = true;
        }
        else {
            ejectActive = false;
        }
        if (el.getLevel() == ElevatorLevel.LEVEL4){
            m_coralGrabberMotor.set(kCoralEjectSpeedLevel4Entry.get());
        }
        else {m_coralGrabberMotor.set(kCoralEjectSpeedEntry.get());}
    }

    // public void setVoltageTest(double voltage) {
    // System.out.println("Setting Coral voltage " + voltage);
    // m_wristMotor.setVoltage(voltage);
    // }

    public void reverseMotor() {
        double voltage = kWristMotorVoltageReverse.get();
        // double speed = kWristMotorSpeedReverse.get();
        // m_wristMotor.set(-speed);
        /*if (m_wristEncoder.getPosition() <= Constants.Coral.kMinEncoderPos) {
            stopMotorWrist();
        } else {
            m_wristMotor.setVoltage(-voltage);
        }*/
        wristDesiredPosition = wristPosition;
        m_wristMotor.setVoltage(-voltage);
    }

    public void forwardMotor() {
        double voltage = kWristMotorVoltageForward.get();
        // double speed = kWristMotorSpeedForward.get();
        /*if (m_wristEncoder.getPosition() >= Constants.Coral.kMaxEncoderPos) {
            stopMotorWrist();
        } else {
            m_wristMotor.setVoltage(voltage);
        }*/
        wristDesiredPosition = wristPosition;
        m_wristMotor.setVoltage(voltage);

        // m_wristMotor.set(speed);
    }

    public void stopMotorWrist() {
        wristDesiredPosition = wristPosition;
        m_wristMotor.setVoltage(0.0);
    }

    public void stopMotorGrabber() {
        m_coralGrabberMotor.set(0.0);
    }

    public void extendManipulator() {
        forwardMotor();
    }

    public void retractManipulator() {
        reverseMotor();
    }

    public Command wristExtendCommand() {
        //return new MoveCoralManipulator(this, true).withTimeout(0.8);
        System.out.println("Extend wrist set desired to " + wristExtendedPosition);
        return new InstantCommand(() -> this.setWristDesiredPosition(wristExtendedPosition));
    }

    public Command wristRetractCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> this.setWristDesiredPosition(0.0)),
            new MoveCoralManipulator(this, false).withTimeout(0.8));
        //System.out.println("Retract wrist set desired to 0.0");
        //return new InstantCommand(() -> this.setWristDesiredPosition(0.0));
    }

    public void setWristDesiredPosition(double desiredPosition) {
        wristDesiredPosition = MathUtil.clamp(desiredPosition, 
            Constants.Coral.kWristMinPosition, Constants.Coral.kWristMaxPosition);
    }

    public boolean detectsCoral() {
        return !m_irSensor.get();
    }

    public void reinit()  {
        boolean irDetected = detectsCoral();
        if (irDetected) {
            state = CoralState.HOLDING;
            el.setLevel(ElevatorLevel.LEVEL1);
            m_coralGrabberMotor.set(0.0);
        } else {
            state = CoralState.WAITING;
        }
        enabled = true;
        wristRolloverCount = 0;
        minEncoderPosition = m_wristEncoder.getPosition();
        wristExtendedPosition = wristExtendedPositionEntry.get();
        minEncoderPositionEntry.set(minEncoderPosition);
        wristExtendedPositionEntry.set(wristExtendedPosition);

        wristDesiredPosition = 0.0;
        lastWristEncoderVal = minEncoderPosition;
    }

    public CoralState getState() {
        return state;
    }

    public void applyWristControl() {
        double wristError = wristDesiredPosition - wristPosition;
        if (wristError > 0) {
            // Wrist should be extended
            wristPidController.setP(kWristPropExtendingEntry.get());
        } else {
            wristPidController.setP(kWristPropRetractingEntry.get());
        }
        if (Math.abs(wristError) <= Constants.Coral.kWristPositionTolerance) {
            m_wristMotor.setVoltage(0.0);
        } else {
            double desiredWristVoltage = -wristPidController.calculate(wristPosition, wristDesiredPosition);
            m_wristMotor.setVoltage(MathUtil.clamp(desiredWristVoltage, -12, 12));
        }
    }

    @Override
    public void periodic() {
        m_wristMotorPublisher.update();
        m_coralGrabberMotorPublisher.update();

        double currentWristEncoderPosition = m_wristEncoder.getPosition();
        m_wristEncoderPub.set(currentWristEncoderPosition);

        boolean irDetected = detectsCoral();
        m_irSensorPub.set(irDetected ? 1.0 : 0.0);
                        
        if (!enabled) {
            return;
        }
        
        if (currentWristEncoderPosition == lastWristEncoderVal) {
            wristCounter++;
            if (wristCounter >= Constants.Coral.wristCounterLimit) {
                wristStopped = true;
            }
        } else {
            wristStopped = false;
            wristCounter = 0;

            // Check for encoder rollover
            if (lastWristEncoderVal > 0.8 && currentWristEncoderPosition < 0.2) {
                // We rolled over while extending
                wristRolloverCount++;
                if (wristRolloverCount > Constants.Coral.kWristMaxRolloverCount) {
                    System.err.println("Invalid increasing wrist rollover past max");
                    wristRolloverCount = Constants.Coral.kWristMaxRolloverCount;
                }
            } else if (lastWristEncoderVal < 0.2 && currentWristEncoderPosition > 0.8) {
                // We rolled over while retracting
                wristRolloverCount--;
                /*if (wristRolloverCount < 0) {
                    System.err.println("Invalid decreasing wrist rollover past 0");
                    wristRolloverCount = 0;
                }*/
            }
            // Calculate the absolute wrist position based on the current encoder value and number of 
            // rollovers, subtracting the min position so that the positions will be zero-based (0 = full retract)
            wristPosition = currentWristEncoderPosition + wristRolloverCount - minEncoderPosition;
        }
        m_wristPositionPub.set(wristPosition);

        // Drive the wrist to the desired position
        applyWristControl();

        if (state == CoralState.WAITING) {
            if (irDetected) {
                state = CoralState.INTAKING;
                counter = m_coralGrabberMotor.getEncoder().getPosition();
                start = counter;
            } else {
                m_coralGrabberMotor.set(0.1);
            }
        } else if (state == CoralState.INTAKING) {
            m_coralGrabberMotor.set(0.1);
            counter = m_coralGrabberMotor.getEncoder().getPosition();
            if (counter >= (start + 4.5)) {
                state = CoralState.HOLDING;
                el.setLevel(ElevatorLevel.LEVEL1);
            } else if (!irDetected) {
                // We thought we were intaking a piece but no longer see one
                state = CoralState.WAITING;
            }
        } else if (state == CoralState.HOLDING) {
            m_coralGrabberMotor.set(0.0);
            if (ejectActive) {
                state = CoralState.EJECTING;
                timer.start();
            } else if (!irDetected) {
                // We thought we were holding a piece but no longer see one
                state = CoralState.WAITING;
                el.setLevel(ElevatorLevel.GROUND);
            }
        } else if (state == CoralState.EJECTING) {
            if (el.getLevel() == ElevatorLevel.LEVEL4){
                m_coralGrabberMotor.set(kCoralEjectSpeedLevel4Entry.get());
            }
            else {m_coralGrabberMotor.set(kCoralEjectSpeedEntry.get());}

            if (timer.get() > 2 && !irDetected) {
                state = CoralState.WAITING;
                ejectActive = false;
            }
        }

        lastWristEncoderVal = currentWristEncoderPosition;
    }

    @Override
    public void makeSafe() {
        Command retract = this.wristRetractCommand();
        retract.schedule();
        stopMotorGrabber();
    }
}
