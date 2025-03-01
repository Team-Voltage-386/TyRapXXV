package frc.robot.Commands.groups;

import frc.robot.Commands.DriveOffset;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreCoralRight extends SequentialCommandGroup {
    public ScoreCoralRight(Drivetrain dt, Limelight ll) {
        super(new DriveOffset(dt, ll, false)); // Calls DriveOffset with isLeft = true
    }
    
}
