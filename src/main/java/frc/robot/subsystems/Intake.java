package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public TalonSRX motor = new TalonSRX(7);

    // public Intake() {

    // }

    public void set(double power) {
        motor.set(TalonSRXControlMode.PercentOutput, power);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
