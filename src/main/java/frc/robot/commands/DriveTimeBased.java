// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.Constants.Auto;

/** An example command that uses an example subsystem. */
public class DriveTimeBased extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive drive;
  private final Timer timer;
  private double t;
  private double v;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveTimeBased(Drive drive, double v, double t) {
    this.drive = drive;
    timer = new Timer();
    this.v = v;
    this.t = t;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetOdometry(Auto.MID_BALL_START_POSE);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.openLoop(v, 0); //change to closedLoop once constants are tuned
    SmartDashboard.putNumber("time: ", timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.openLoop(0, 0);
    timer.stop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(t);
  }
}
