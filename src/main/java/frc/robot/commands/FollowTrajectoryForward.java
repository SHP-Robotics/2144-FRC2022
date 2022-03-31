package frc.robot.commands;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Drive.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;

public class FollowTrajectoryForward extends CommandBase {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drive drive;
  private Trajectory trajectory;
  private RamseteController controller;
  private Timer timer;

  public FollowTrajectoryForward(Drive drive) {
    this.drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.disableRamp();

    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        drive.getFeedforward(),
        drive.getKinematics(),
        kNominalVoltage - 2);

    TrajectoryConfig config = new TrajectoryConfig(
        kMaxVelocity,
        kMaxAcceleration)
            .setKinematics(drive.getKinematics())
            .addConstraint(autoVoltageConstraint);

    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(2, 0, Rotation2d.fromDegrees(0));
    ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
    // interiorWaypoints.add(new Translation2d(0.5, 0.1));
    // interiorWaypoints.add(new Translation2d(1.5, 0.3));

    trajectory = TrajectoryGenerator.generateTrajectory(
        startPose,
        interiorWaypoints,
        endPose,
        config);
    drive.getField().getObject("traj").setTrajectory(trajectory);

    // potentential jerking if something doesnt match up properly
    drive.resetOdometry(trajectory.getInitialPose());

    controller = new RamseteController();

    timer = new Timer();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = trajectory.sample(timer.get());
    ChassisSpeeds adjustedSpeeds = controller.calculate(drive.getPose(), goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = drive.getKinematics().toWheelSpeeds(adjustedSpeeds);

    drive.closedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    drive.stop();
    drive.enableRamp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getTotalTimeSeconds();
  }
}
