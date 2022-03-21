package frc.robot.subsystems;

import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.kNominalVoltage;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Drive extends SubsystemBase {
  private DifferentialDrive driveBase;

  public final WPI_TalonFX[] motors;
  // new WPI_TalonFX(0), // Left Front
  // new WPI_TalonFX(1), // Left Back
  // new WPI_TalonFX(2), // Right Front
  // new WPI_TalonFX(3) // Right Back

  private final MotorControllerGroup leftDrive;
  private final MotorControllerGroup rightDrive;

  private final AHRS navx;

  private final SimpleMotorFeedforward feedforward;

  // private double previousAngle = 0;

  // private final PIDController pid = new PIDController(0.01, 0, 0);

  private final DifferentialDriveOdometry odometry;
  private final DifferentialDriveKinematics kinematics;

  public Drive() {
    motors = new WPI_TalonFX[4];

    for (int i = 0; i < 4; i++) {
      motors[i] = new WPI_TalonFX(i);
      motors[i].setNeutralMode(NeutralMode.Coast);
    }

    this.enableRamp();

    leftDrive = new MotorControllerGroup(motors[0], motors[1]);
    rightDrive = new MotorControllerGroup(motors[2], motors[3]);

    leftDrive.setInverted(false);
    rightDrive.setInverted(true);

    driveBase = new DifferentialDrive(leftDrive, rightDrive);

    feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    navx = new AHRS(SPI.Port.kMXP);

    odometry = new DifferentialDriveOdometry(navx.getRotation2d());
    kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
  }

  public void enableRamp() {
    for (WPI_TalonFX motor : motors)
      motor.configOpenloopRamp(0.8);
  }

  public void disableRamp() {
    for (WPI_TalonFX motor : motors)
      motor.configOpenloopRamp(0);
  }

  public void openLoop(double straight, double turn) {
    // prevent stick drift
    if (straight < 0.1 && straight > -0.1)
      straight = 0;
    if (turn < 0.1 && turn > -0.1)
      turn = 0;

    // if moving forward more than 30%, halve turn bias
    if (straight > 0.3)
      turn /= 2;

    // double driftCompensation = 0;
    // double angle = navx.getAngle();
    // if (rightX < 0.1 && rightX > -0.1) { // driving forward
    // rightX = 0;
    // driftCompensation = pid.calculate(angle, previousAngle);
    // } else previousAngle = angle; // turning
    // SmartDashboard.putNumber("drift compensation", driftCompensation);

    straight = MathUtil.clamp(straight, -kForwardThreshold, kForwardThreshold);
    turn = MathUtil.clamp(Math.pow(turn, 3) * kTurningSensitivity, -kTurnThreshold, kTurnThreshold);

    // rightX = Math.pow(rightX, 5) / Math.abs(Math.pow(rightX, 3));

    double leftSpeed = straight - turn;
    double rightSpeed = straight + turn;

    leftSpeed = MathUtil.clamp(leftSpeed, -kForwardThreshold, kForwardThreshold);
    rightSpeed = MathUtil.clamp(rightSpeed, -kForwardThreshold, kForwardThreshold);

    // System.out.println(leftSpeed);

    // System.out.println(rightSpeed);

    // driveBase.tankDrive(leftRamp.calculate(leftSpeed + driftCompensation),
    // rightRamp.calculate(rightSpeed));
    driveBase.tankDrive(leftSpeed, rightSpeed);
  }

  public void closedLoop(double leftMetersPerSecond, double rightMetersPerSecond) {
    leftDrive.setVoltage(feedforward.calculate(leftMetersPerSecond) * kNominalVoltage);
    leftDrive.setVoltage(feedforward.calculate(rightMetersPerSecond) * kNominalVoltage);
  }

  public void stop() {
    leftDrive.set(0);
    rightDrive.set(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftAverageTicksPerSecond = (motors[0].getSelectedSensorVelocity() + motors[1].getSelectedSensorVelocity())
        / 2 * 10;
    double rightAverageTicksPerSecond = (motors[2].getSelectedSensorVelocity() + motors[3].getSelectedSensorVelocity())
        / 2 * 10;
    return new DifferentialDriveWheelSpeeds(
        leftAverageTicksPerSecond * kMetersPerTick,
        rightAverageTicksPerSecond * kMetersPerTick);
  }

  public double getHeading() {
    return navx.getAngle();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(rightVolts);
    // driveBase.feed();
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public void resetOdometry(Pose2d position)
  {
    odometry.resetPosition(position, position.getRotation());
  }

  public void updateOdometry()
  {
    odometry.update(
        navx.getRotation2d(),
        (motors[0].getSelectedSensorPosition() + motors[1].getSelectedSensorPosition()) / 2,
        (motors[2].getSelectedSensorPosition() + motors[3].getSelectedSensorPosition()) / 2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("navx angle", navx.getAngle());
    updateOdometry();

    SmartDashboard.putNumber("navx pitch", navx.getPitch());
    SmartDashboard.putNumber("navx roll", navx.getRoll());
    SmartDashboard.putNumber("navx yaw", navx.getYaw());
    SmartDashboard.putNumber("navx angle", navx.getAngle());

  }
}
