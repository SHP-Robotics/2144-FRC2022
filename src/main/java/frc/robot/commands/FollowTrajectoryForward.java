package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Example;

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
import edu.wpi.first.wpilibj.Timer;

public class FollowTrajectoryForward extends CommandBase {
  //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /*
  var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(
      DriveConstants.ksVolts,
      DriveConstants.kvVoltSecondsPerMeter,
      DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      10);
  */
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
    timer = new Timer();
    timer.start();

    TrajectoryConfig config =
        new TrajectoryConfig(
                4,
                4)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(drive.getKinematics());
            // Apply the voltage constraint
            //.addConstraint(autoVoltageConstraint);

    Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(2, 0, Rotation2d.fromDegrees(0));
    var interiorWaypoints = new ArrayList<Translation2d>();
    //interiorWaypoints.add(new Translation2d(0.5, 0.1);
    //interiorWaypoints.add(new Translation2d(1.5, 0.3));
    drive.resetOdometry(trajectory.getInitialPose());

    trajectory = TrajectoryGenerator.generateTrajectory(
        startPose, 
        interiorWaypoints, 
        endPose, 
        config);
    controller = new RamseteController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.updateOdometry();

    Trajectory.State goal = trajectory.sample(timer.get());
    ChassisSpeeds adjustedSpeeds = controller.calculate(drive.getPose(), goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = drive.getKinematics().toWheelSpeeds(adjustedSpeeds);
    drive.drive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>trajectory.getTotalTimeSeconds();
  }
}
