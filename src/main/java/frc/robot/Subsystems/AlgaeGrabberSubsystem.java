package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.Elastic;
import frc.robot.Utils.Elastic.Notification;
import frc.robot.Utils.Elastic.Notification.NotificationLevel;
import frc.robot.Utils.MotorPublisher;
import frc.robot.Utils.SafeableSubsystem;

public class AlgaeGrabberSubsystem extends SafeableSubsystem {
    private final Value pneumaticRetract = Value.kReverse; // this retracts the grabber
    private final Value pneumaticExtend = Value.kForward; // extend grabber
    private final Value pneumaticOff = Value.kOff;

    private final SparkMax retrieval_motor;
    private final DoubleSolenoid raise_pneumatics_solenoid;

    private final NetworkTable table;

    private final MotorPublisher motorPublisher;

    private final BooleanPublisher table_motor_stalled;

    private final StringPublisher table_solenoid_state;

    /**
     * Should the robot extend the algae retriever and run the motor?
     */
    private final BooleanEntry table_should_retrieve;

    /** Creates a new AlgaeGrabber. */
    public AlgaeGrabberSubsystem(NetworkTableInstance nt) {
        retrieval_motor = new SparkMax(Constants.MechID.kAlgaeMotorCanId, MotorType.kBrushless);

        raise_pneumatics_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                Constants.AlgaeGrabber.kSolenoidCANID1,
                Constants.AlgaeGrabber.kSolenoidCANID2);

        table = nt.getTable(getName());

        motorPublisher = new MotorPublisher(retrieval_motor, table, "motor");

        table_motor_stalled = table.getBooleanTopic("motor_stalled").publish();

        table_solenoid_state = table.getStringTopic("solenoid_state").publish();

        table_should_retrieve = table.getBooleanTopic("should_retrieve").getEntry(false);
        // needed to make the entry show up in elastic
        table_should_retrieve.set(false);

        // new Trigger(() -> table_should_retrieve.get()).and(() ->
        // !isMotorStalled()).onTrue(runOnce(() -> {
        // retrieval_motor.set(Constants.AlgaeGrabber.kMotorSpeed);
        // raise_pneumatics_solenoid.set(pneumaticOut);
        // )).onFalse(runOnce(() -> {
        // table_should_retrieve.set(false);
        // retrieval_motor.set(0.0);
        // raise_pneumatics_solenoid.set(pneumaticIn);
        // }));
    }

    // public Command toggleRetriever() {
    // return runOnce(() -> {
    // if (table_should_retrieve.get()){
    // raise_pneumatics_solenoid.set(pneumaticRetract);
    // retrieval_motor.set(0.0);
    // }
    // else {
    // raise_pneumatics_solenoid.set(pneumaticExtend);
    // retrieval_motor.set(-Constants.AlgaeGrabber.kIntakeSpeed);
    // }
    //
    // table_should_retrieve.set(!table_should_retrieve.get());
    // });
    // }

    public void extendGrabber() {
        raise_pneumatics_solenoid.set(pneumaticExtend);
        retrieval_motor.set(-Constants.AlgaeGrabber.kIntakeSpeed);
    }

    public void retractGrabber() {
        raise_pneumatics_solenoid.set(pneumaticRetract);
        retrieval_motor.set(0.0);
    }

    public void stopMotor() {
        retrieval_motor.set(0);
    }

    public void ejectAlgae() {
        retrieval_motor.set(Constants.AlgaeGrabber.kEjectSpeed);
    }

    public boolean isMotorStalled() {
        return retrieval_motor.getOutputCurrent() > Constants.AlgaeGrabber.kMotorCurrentThreshold;
    }

    public void makeSafe() {
        stopMotor();
        retractGrabber();
    }

    @Override
    public void periodic() {
        motorPublisher.update();

        table_motor_stalled.set(isMotorStalled());

        Value state = raise_pneumatics_solenoid.get();
        table_solenoid_state.set(state == pneumaticRetract ? "out" : state == pneumaticExtend ? "in" : "off");

        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
