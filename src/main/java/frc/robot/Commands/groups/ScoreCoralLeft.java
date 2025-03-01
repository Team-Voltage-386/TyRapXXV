package frc.robot.Commands.groups;

import frc.robot.Commands.DriveOffset;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreCoralLeft extends SequentialCommandGroup {
    public ScoreCoralLeft(Drivetrain dt, Limelight ll) {

        addCommands(
            new DriveOffset(dt, ll, true)  // Step 1: Drive offset left
            //new SomeOtherCommand(),         // Step 2: Replace with actual scoring command
            //new AnotherCommand()            // Step 3: Replace with any additional actions
        );
    }
}
