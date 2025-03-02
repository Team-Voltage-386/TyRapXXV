package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralSubsystem;

public class EjectCoral extends Command {
    CoralSubsystem co;
    private boolean cancelFlag;

    public EjectCoral(CoralSubsystem co) {
        this.co = co;
        addRequirements(co);
    }

    @Override
    public void initialize() {
        co.ejectCoral();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public void stopComand(){
        cancelFlag = true;
    }
}

