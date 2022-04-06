package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
                // new WaitCommand(1),
                new ConditionalCommand(new ShootTwoBalls(indexer), new ShootBall(indexer), indexer::twoBallsIndexed),
                new InstantCommand(() -> indexer.setIndexer(0), indexer));
                // new WaitCommand(0.5),
                // new InstantCommand(() -> flywheel.setDesiredVelocityRPS(0), flywheel));
    }
}
