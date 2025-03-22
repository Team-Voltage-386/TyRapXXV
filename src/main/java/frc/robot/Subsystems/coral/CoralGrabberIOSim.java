package frc.robot.Subsystems.coral;

import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CoralGrabberIOSim implements CoralGrabberIO {
    private final IntakeSimulation intakeSim;
    private final DCMotor gearbox = DCMotor.getNeo550(1);
    // TODO: find actual gearing
    private final DCMotorSim motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 4.0, 6.75), gearbox);

    private final AbstractDriveTrainSimulation driveSim;

    public CoralGrabberIOSim(AbstractDriveTrainSimulation driveSim) {
        intakeSim = new IntakeSimulation("Coral", driveSim,
                new Triangle(new Vector2(0, 0), new Vector2(0.2, 0), new Vector2(0, 0.2)), 1);
        intakeSim.startIntake();

        this.driveSim = driveSim;
    }

    public IntakeSimulation getIntakeSim() {
        return intakeSim;
    }

    @Override
    public void updateInputs(CoralGrabberIOInputs inputs) {
        inputs.relativeEncoderPosition = new Rotation2d(motor.getAngularPosition());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setInputVoltage(voltage.in(Units.Volts));
        if (voltage.in(Units.Volts) > 4.0 && intakeSim.getGamePiecesAmount() > 0) {
            // we're ejecting with a coral
            if (intakeSim.obtainGamePieceFromIntake()) {
                // if it returned true, the coral was removed from the simulated intake
                // TODO: tune sim
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                // Obtain robot position from drive simulation
                                driveSim.getSimulatedDriveTrainPose().getTranslation(),
                                // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                                new Translation2d(0.46, 0),
                                // Obtain robot speed from drive simulation
                                driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                // Obtain robot facing from drive simulation
                                driveSim.getSimulatedDriveTrainPose().getRotation(),
                                // The height at which the coral is ejected
                                Units.Meters.of(2.10),
                                // The initial speed of the coral
                                Units.MetersPerSecond.of(2),
                                // The coral is ejected at a 35-degree slope
                                Units.Degrees.of(-35)));
            }
        }
    }

    @Override
    public void setSpeed(double speed) {
        // I don't think this actually how it works, but whatever
        setVoltage(Units.Volts.of(speed * 12.0));
    }
}
