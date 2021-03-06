package frc.robot.commands;

import static frc.robot.Constants.Flywheel.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Vision;

public class SpinFlywheel extends CommandBase {
    private final Flywheel flywheel;
    private final Vision vision;

    public SpinFlywheel(Flywheel flywheel, Vision vision) {
        this.flywheel = flywheel;
        this.vision = vision;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(flywheel, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        flywheel.enableRamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        flywheel.setDesiredVelocityRPS(vision.isTarget() ? Interpolation.table.get(vision.getDistanceInches())
                : kDefaultRPS);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        flywheel.disableRamp();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return flywheel.isAtSetpoint();
    }
}
