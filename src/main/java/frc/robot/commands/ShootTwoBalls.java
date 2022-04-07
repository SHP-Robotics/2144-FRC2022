package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;

public class ShootTwoBalls extends SequentialCommandGroup {
    public ShootTwoBalls(Indexer indexer) {
        addCommands(
            new ShootBall(indexer),
            new WaitCommand(1.5),
            new ShootBall(indexer)
        );
    }
}
