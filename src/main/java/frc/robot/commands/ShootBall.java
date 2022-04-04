package frc.robot.commands;

import static frc.robot.Constants.Indexer.kIndexerSpeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class ShootBall extends CommandBase {
    private final Indexer indexer;

    public ShootBall(Indexer indexer) {
        this.indexer = indexer;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(indexer);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        indexer.setIndexer(kIndexerSpeed);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !indexer.ballIndexedFirst();
    }
}
