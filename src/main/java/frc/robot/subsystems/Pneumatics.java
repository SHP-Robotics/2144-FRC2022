// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private DoubleSolenoid doubleSolenoid;

  public Pneumatics() {
    // doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    doubleSolenoid.set(Value.kOff);
  }

  public void forward() {
    doubleSolenoid.set(Value.kForward);
  }

  public void reverse() {
    doubleSolenoid.set(Value.kReverse);
  }

  public void off() {
    doubleSolenoid.set(Value.kOff);
  }

  public Value get() {
    return doubleSolenoid.get();
  }

  public void toggle() {
    doubleSolenoid.toggle();
  }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
}
