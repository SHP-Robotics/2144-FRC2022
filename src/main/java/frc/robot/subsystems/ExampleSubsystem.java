// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private CANSparkMax motor = new CANSparkMax(6, MotorType.kBrushed);

  // public ExampleSubsystem() {

  // }

  public void set(double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("mag enc", motor.getEncoder().getPosition());
  }

  // @Override
  // public void simulationPeriodic() {
  // // This method will be called once per scheduler run during simulation
  // }
}
