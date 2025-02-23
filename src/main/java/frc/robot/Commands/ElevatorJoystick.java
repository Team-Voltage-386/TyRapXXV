package frc.robot.Commands;

import java.net.NetworkInterface;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Controller;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorJoystick extends Command {
    NetworkTable table;

    ElevatorSubsystem el;

    private final XboxController m_controller = new XboxController(Controller.kManipControllerID);
  
    protected final DoubleEntry multiplier;
  
    public ElevatorJoystick(ElevatorSubsystem el, NetworkTableInstance nt) {
        this.el = el;
        table = nt.getTable(getName());

        multiplier = table.getDoubleTopic("multiplier").getEntry(3.5);
        multiplier.set(3.5);

        System.out.println("ElevatorJoystick command initialized");
    }

    @Override
    public void initialize() {
        this.el.setTestMode(true);
    }

    @Override
    public void execute() {
        double elSpeed = m_controller.getLeftY() * multiplier.get();
        el.setVoltageTest(elSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("ElevatorJoystick command interrupted");
        }
        this.el.setTestMode(false);
    }
}
