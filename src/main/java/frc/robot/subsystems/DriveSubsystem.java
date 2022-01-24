package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private DifferentialDrive driveBase;
  private CANSparkMax[] motors = {
      new CANSparkMax(1, MotorType.kBrushless), // Left Front
      new CANSparkMax(2, MotorType.kBrushless), // Left Back
      new CANSparkMax(3, MotorType.kBrushless), // Right Front
      new CANSparkMax(4, MotorType.kBrushless) // Right Back
  };
  private MotorControllerGroup leftDrive = new MotorControllerGroup(motors[0], motors[1]);
  private MotorControllerGroup rightDrive = new MotorControllerGroup(motors[2], motors[3]);
  private int mode = 1;

  public DriveSubsystem() {
    // Constants.Drive.kMotors[0].setInverted(true);
    // Constants.Drive.kMotors[1].setInverted(true);
    leftDrive.setInverted(true);
    rightDrive.setInverted(false);
    driveBase = new DifferentialDrive(leftDrive, rightDrive);
    // driveBase = new MecanumDrive(
    // Constants.Drive.kMotors[0],
    // Constants.Drive.kMotors[1],
    // Constants.Drive.kMotors[2],
    // Constants.Drive.kMotors[3]);
  }

  public void drive(double leftY, double rightY, double rightX) {
    // if (rightX < 0.1 && rightX > -0.1)
    //   rightX = 0;

    // rightX = Math.pow(rightX, 3);
    // rightX *= 1.25;
    rightX = Math.pow(rightX, 5) / Math.abs(Math.pow(rightX, 3));

    double leftSpeed = mode == 0 ? leftY : leftY - rightX;
    double rightSpeed = mode == 0 ? rightY : leftY + rightX;

    if (leftSpeed > 1.0) leftSpeed = 1.0;
    if (leftSpeed < -1.0) leftSpeed = -1.0;
    if (rightSpeed > 1.0) rightSpeed = 1.0;
    if (rightSpeed < -1.0) rightSpeed = -1.0;

    System.out.println(leftSpeed);
    System.out.println(rightSpeed);

    driveBase.tankDrive(leftSpeed * 2.0 / 3.0, rightSpeed * 2.0 / 3.0);
    // *
  }

  public void switchMode() {
    mode = mode == 0 ? 1 : 0;
    System.out.println("Switched to: " + mode);
  }

  // @Override
  // public void periodic() {
  // // This method will be called once per scheduler run
  // }
}
