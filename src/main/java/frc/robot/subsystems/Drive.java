package frc.robot.subsystems;

import static frc.robot.Constants.Drive.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class Drive extends SubsystemBase {
  private DifferentialDrive driveBase;

  public WPI_TalonFX[] motors;
  // new WPI_TalonFX(0), // Left Front
  // new WPI_TalonFX(1), // Left Back
  // new WPI_TalonFX(2), // Right Front
  // new WPI_TalonFX(3) // Right Back

  private MotorControllerGroup leftDrive;
  private MotorControllerGroup rightDrive;
 
  private AHRS navx;

  // private double previousAngle = 0;

  // private final PIDController pid = new PIDController(0.01, 0, 0);

  private final DifferentialDriveOdometry odometry;
  public Drive() {
    motors = new WPI_TalonFX[4];

    for (int i = 0; i < 4; i++) {
      motors[i] = new WPI_TalonFX(i);
      motors[i].configOpenloopRamp(0.8);
      motors[i].setNeutralMode(NeutralMode.Coast);
    }

    leftDrive = new MotorControllerGroup(motors[0], motors[1]);
    rightDrive = new MotorControllerGroup(motors[2], motors[3]);

    leftDrive.setInverted(false);
    rightDrive.setInverted(true);

    driveBase = new DifferentialDrive(leftDrive, rightDrive);

    navx = new AHRS(SPI.Port.kMXP);
    odometry = new DifferentialDriveOdometry(navx.getRotation2d());
  }

  public void drive(double leftY, double rightX) {
    if (leftY < 0.1 && leftY > -0.1)
      leftY = 0;
    if (rightX < 0.1 && rightX > -0.1)
      rightX = 0;

    // double driftCompensation = 0;
    // double angle = navx.getAngle();
    // if (rightX < 0.1 && rightX > -0.1) { // driving forward
    // rightX = 0;
    // driftCompensation = pid.calculate(angle, previousAngle);
    // } else previousAngle = angle; // turning

    leftY = MathUtil.clamp(leftY, -kForwardThreshold, kForwardThreshold);
    rightX = MathUtil.clamp(Math.pow(rightX, 3) * kTurningSensitivity, -kTurnThreshold, kTurnThreshold);

    // rightX = Math.pow(rightX, 5) / Math.abs(Math.pow(rightX, 3));

    double leftSpeed = leftY - rightX;
    double rightSpeed = leftY + rightX;

    leftSpeed = MathUtil.clamp(leftSpeed, -kForwardThreshold, kForwardThreshold);
    rightSpeed = MathUtil.clamp(rightSpeed, -kForwardThreshold, kForwardThreshold);

    // System.out.println(leftSpeed);

    // System.out.println(rightSpeed);

    // driveBase.tankDrive(leftRamp.calculate(leftSpeed + driftCompensation),
    // rightRamp.calculate(rightSpeed));
    driveBase.tankDrive(leftSpeed, rightSpeed);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      (motors[0].getSelectedSensorVelocity() + motors[1].getSelectedSensorVelocity()) / 2, 
      (motors[2].getSelectedSensorVelocity() + motors[3].getSelectedSensorVelocity()) / 2);
      
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
    driveBase.feed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("navx angle", navx.getAngle());
    odometry.update(
      navx.getRotation2d(), 
      (motors[0].getSelectedSensorPosition() + motors[1].getSelectedSensorPosition()) / 2, 
      (motors[2].getSelectedSensorPosition() + motors[3].getSelectedSensorPosition()) / 2);
  }
}
