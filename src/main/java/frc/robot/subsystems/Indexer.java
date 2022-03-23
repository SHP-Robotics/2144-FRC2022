package frc.robot.subsystems;

import static frc.robot.Constants.Indexer.*;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final TalonFX shooterFeeder = new TalonFX(8);
    // private final SlewRateLimiter ramp = new SlewRateLimiter(0.8);
    private final DigitalInput firstSwitch = new DigitalInput(0);
    private final DigitalInput secondSwitch = new DigitalInput(1);
    private final DigitalInput intakeSwitch = new DigitalInput(2);

    private boolean ballMoving = false;

    public Indexer() {
        shooterFeeder.configVoltageCompSaturation(kVoltageSaturation);
        shooterFeeder.configVoltageMeasurementFilter(kVoltageMeasurementSamples);
        // motor.setInverted(true);
        // motor.setNeutralMode(NeutralMode.Coast);
    }

    public void set(double power) {
        // motor.set(TalonFXControlMode.PercentOutput, ramp.calculate(power));
    }

    public void moveIndexedBallIntoTurret() {
        shooterFeeder.set(TalonFXControlMode.PercentOutput, kDefaultSpeed);
    }

    public void stopShooting() {
        shooterFeeder.set(TalonFXControlMode.PercentOutput, 0);
    }

    public boolean isBallIndexedFirst() {
        return !firstSwitch.get();
    }

    public boolean isBallindexedSecond() {
        return !secondSwitch.get();
    }

    public boolean isBallIntaked() {
        return !intakeSwitch.get();
    }

    @Override
    public void periodic() {
        boolean ballIndexedFirst = isBallIndexedFirst();
        boolean ballIndexedSecond = isBallindexedSecond();
        boolean ballIntaked = isBallIntaked();
        SmartDashboard.putBoolean("is ball indexed?", ballIndexedFirst);

        // ballMoving ? moveBallToSecondIndex : stopMovingBall

        // if no ball in first index and ball in second index
        if (!ballIndexedFirst && ballIndexedSecond) {
            // move ball into first index
        }

        // if a ball is found in second index and a ball is supposed to be moving
        if (ballIndexedSecond && ballMoving) {
            // stop moving ball
            ballMoving = false;
        }

        if (ballIntaked) {
            // if two balls indexed
            if (ballIndexedFirst && ballIndexedSecond) {
                // send ball back
            } else {
                // send ball forward
                ballMoving = true;
            }
        }
    }
}
