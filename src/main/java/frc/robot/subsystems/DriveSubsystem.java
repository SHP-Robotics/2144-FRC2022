package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private DifferentialDrive driveBase;

  public WPI_TalonFX[] motors = new WPI_TalonFX[4];
  // new WPI_TalonFX(0), // Left Front
  // new WPI_TalonFX(1), // Left Back
  // new WPI_TalonFX(2), // Right Front
  // new WPI_TalonFX(3) // Right Back

  private MotorControllerGroup leftDrive;
  private MotorControllerGroup rightDrive;

  private int mode = 1;

  public DriveSubsystem() {
    for (int i = 0; i < 4; i++) {
      motors[i] = new WPI_TalonFX(i);
    }

    for (WPI_TalonFX motor : motors) {
      motor.configOpenloopRamp(0.8);
      motor.setNeutralMode(NeutralMode.Brake);
    }

    leftDrive = new MotorControllerGroup(motors[0], motors[1]);
    rightDrive = new MotorControllerGroup(motors[2], motors[3]);

    leftDrive.setInverted(false);
    rightDrive.setInverted(true);

    driveBase = new DifferentialDrive(leftDrive, rightDrive);
  }

  public void drive(double leftY, double rightY, double rightX) {    
    if (rightX < 0.1 && rightX > -0.1)
      rightX = 0;

    double steeringSensitivity = 1.5;
    rightX = Math.pow(rightX, 3) * steeringSensitivity;

    if (rightX > 0.5) rightX = 0.5;
    if (rightX < -0.5) rightX = -0.5;

    // rightX = Math.pow(rightX, 5) / Math.abs(Math.pow(rightX, 3));

    double leftSpeed = mode == 0 ? leftY : leftY - rightX;
    double rightSpeed = mode == 0 ? rightY : leftY + rightX;

    if (leftSpeed > 1.0)
      leftSpeed = 1.0;
    if (leftSpeed < -1.0)
      leftSpeed = -1.0;
    if (rightSpeed > 1.0)
      rightSpeed = 1.0;
    if (rightSpeed < -1.0)
      rightSpeed = -1.0;

    // System.out.println(leftSpeed);
    // System.out.println(rightSpeed);

    driveBase.tankDrive(leftSpeed, rightSpeed);
  }

  public void switchMode() {
    mode = mode == 0 ? 1 : 0;
    System.out.println("Switched to: " + mode);
  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run
  }
}
