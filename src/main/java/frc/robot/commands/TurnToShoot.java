// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class TurnToShoot extends CommandBase {
  private final Drive drive;
  private final Vision vision;

  public TurnToShoot(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;
    
    addRequirements(drive, vision);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.closedLoop(0.5, -0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.isTarget();
  }
}
