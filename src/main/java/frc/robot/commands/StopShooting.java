package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;

public class StopShooting extends CommandBase {
    private final Flywheel flywheel;
    private final Indexer indexer;


    public StopShooting(Flywheel flywheel, Indexer indexer) {
        this.flywheel = flywheel;
        this.indexer = indexer;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(flywheel, indexer);
    }

    // Called when the command is initially scheduled.
    // @Override
    // public void initialize() {
    // }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        flywheel.setVelocityRotationsPerSecond(0);
        indexer.stopShooting();
    }

    // Called once the command ends or is interrupted.
    // @Override
    // public void end(boolean interrupted) {
    // }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
