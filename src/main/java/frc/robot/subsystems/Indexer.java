package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private TalonFX motor = new TalonFX(8);
    private SlewRateLimiter ramp = new SlewRateLimiter(0.8);

    public Indexer() {
        motor.setInverted(true);
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public void set(double power) {
        motor.set(TalonFXControlMode.PercentOutput, ramp.calculate(power));
    }

    public void moveIndexedBallIntoTurret() {
        motor.set(TalonFXControlMode.PercentOutput, Constants.Indexer.kDefaultSpeed);
    }

    public void stop() {
        motor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public boolean isBallIndexed() {
        return false; // for now
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
