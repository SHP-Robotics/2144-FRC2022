package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Turret.*;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final TalonFX motor = new TalonFX(6);

    private boolean closedLoop = false;
    // private double panPower = 0.1;

    /**
     * Creates a new Turret using built-in PID and motion profiling.
     */
    public Turret() {
        motor.config_kP(0, kP);
        motor.config_kI(0, kI);
        motor.config_kD(0, kD);

        // motor.config_kP(0, 0);
        // motor.config_kI(0, 0);
        // motor.config_kD(0, 0);

        motor.configMotionCruiseVelocity(kCruiseVelocity);
        motor.configMotionAcceleration(kAcceleration);

        motor.setInverted(true);
    }

    /**
     * @return The encoder position of the motor.
     */
    public double getEncoderPosition() {
        return motor.getSelectedSensorPosition();
    }

    /**
     * @return The angle the turret is at in degrees.
     */
    public double getDegrees() {
        // turret positon / ticks per revolution * 360
        return this.getEncoderPosition() * ratio / kFalconTicksPerRevolution * 360;
    }

    /**
     * @return The angle the turret is at in radians.
     */
    public double getRadians(double pulses) {
        return Math.toRadians(this.getDegrees());
    }

    /**
     * @param degrees Turret angle in degrees.
     * @return The encoder position for a given angle in degrees.
     */
    public double convertDegreesToTicks(double degrees) {
        // degrees / 360 * ticks per turret revolution
        return degrees / 360 * kFalconTicksPerRevolution / ratio;
    }

    /**
     * Resets the encoder position to 0.
     */
    public void resetPosition() {
        motor.setSelectedSensorPosition(0);
    }

    /**
     * Toggles between open-loop and closed-loop control.
     */
    public void toggleLoop() {
        closedLoop = !closedLoop;
    }

    // public void panTurret() {
    // if (this.getDegrees() >= kThresholdDegrees)
    // panPower = -Math.abs(panPower);
    // else if (this.getDegrees() <= -kThresholdDegrees)
    // panPower = Math.abs(panPower);
    // motor.set(TalonFXControlMode.PercentOutput, panPower);
    // }

    /**
     * Adjusts the turret angle by the given degrees. Only adjusts if there is a
     * target.
     * 
     * @param isTarget Whether or not the turret has a target.
     * @param degrees  Turret angle in degrees.
     */
    public void adjust(boolean isTarget, double degrees) {
        if (!closedLoop) {
            motor.set(TalonFXControlMode.PercentOutput, 0);
            return;
        }

        if (!isTarget) {
            // this.panTurret();
            return;
        }

        double desiredDegrees = this.getDegrees() + degrees;
        if (desiredDegrees > kThresholdDegrees)
            desiredDegrees = kThresholdDegrees;
        else if (desiredDegrees < -kThresholdDegrees)
            desiredDegrees = -kThresholdDegrees;

        motor.set(TalonFXControlMode.MotionMagic, this.convertDegreesToTicks(desiredDegrees));
    }

    /**
     * Open-loop control for the turret.
     * 
     * @param speed The speed to set, between 1.0 and -1.0.
     */
    public void openLoop(double speed) {
        if (closedLoop)
            closedLoop = false;

        double degrees = this.getDegrees();
        if ((degrees > kThresholdDegrees && speed > 0) || (degrees < -kThresholdDegrees && speed < 0)) {
            motor.set(TalonFXControlMode.PercentOutput, 0);
        } else
            motor.set(TalonFXControlMode.PercentOutput, speed);
    }

    /**
     * @return Whether or not the turret is in closed-loop control.
     */
    public boolean isClosedLoop() {
        return closedLoop;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Driver Override", !closedLoop);
        SmartDashboard.putNumber("Turret Degrees", this.getDegrees());
    }
}
