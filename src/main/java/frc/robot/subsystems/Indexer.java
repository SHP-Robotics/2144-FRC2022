package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private TalonFX motor = new TalonFX(8);
    private SlewRateLimiter ramp = new SlewRateLimiter(0.8);


    public Indexer() {
        motor.setInverted(true);
    }

    public void set(double power) {
        motor.set(TalonFXControlMode.PercentOutput, ramp.calculate(power));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
