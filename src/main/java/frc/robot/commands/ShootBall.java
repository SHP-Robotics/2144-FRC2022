package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Vision;

public class ShootBall extends CommandBase {
    private final Indexer indexer;

    public ShootBall(Indexer indexer) {
        this.indexer = indexer;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(indexer);
    }

    // Called when the command is initially scheduled.
    // @Override
    // public void initialize() {
    // }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        indexer.moveIndexedBallIntoTurret();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !indexer.ballIndexed();
    }
}
