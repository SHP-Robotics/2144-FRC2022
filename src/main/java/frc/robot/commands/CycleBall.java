package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Vision;

public class CycleBall extends SequentialCommandGroup {
    public CycleBall(Flywheel flywheel, Vision vision, Indexer indexer) {
        addCommands(
                new SpinFlywheel(flywheel, vision),
                new ShootBall(indexer),
                new WaitCommand(2),
                new InstantCommand(() -> {
                    flywheel.setDesiredVelocityRPS(0);
                    indexer.stopShooting();
                }, flywheel, indexer));
    }
}
