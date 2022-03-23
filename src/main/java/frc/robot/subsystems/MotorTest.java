package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorTest extends SubsystemBase {
    private TalonFX motor = new TalonFX(8);

    public MotorTest() {
        // motor.setInverted(true);
    }

    public void set(double power) {
        motor.set(TalonFXControlMode.PercentOutput, power);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
}
