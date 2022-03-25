package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorTest extends SubsystemBase {
    private TalonFX motor = new TalonFX(5);
    private Orchestra orchestra = new Orchestra();

    public MotorTest() {
        orchestra.addInstrument(motor);
        // motor.setInverted(true);
    }

    public void playSong() {
        orchestra.loadMusic("happybday.chrp");
        orchestra.play();
    }

    public void set(double power) {
        motor.set(TalonFXControlMode.PercentOutput, power);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
}
