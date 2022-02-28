package frc.robot.subsystems;

import static frc.robot.Constants.Turret.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class TurretSubsystem extends SubsystemBase implements Loggable {
    private final TalonSRX motor = new TalonSRX(6);
    private final double ratio = 1.0 / 12.7; // input : output

    @Log(name = "auto enabled")
    private boolean isAutoEnabled = false;

    private double panPower = 0.2;

    private final ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD,
            new TrapezoidProfile.Constraints(1, 1));

    public TurretSubsystem() {
        // motor.config_kP(0, kP);
        // motor.config_kI(0, kI);
        // motor.config_kD(0, kD);

        motor.config_kP(0, 0);
        motor.config_kI(0, 0);
        motor.config_kD(0, 0);
    }

    @Log(name = "turret encoder")
    public double getEncoderPosition() {
        return motor.getSelectedSensorPosition();
    }

    @Log(name = "turret degrees")
    public double getDegrees() {
        // turret positon / ticks per revolution * 360
        return this.getEncoderPosition() * ratio / kTicksPerRevolution * 360;
    }

    public double getRadians(double pulses) {
        return Math.toRadians(this.getDegrees());
    }

    public double convertDegreesToTicks(double degrees) {
        // degrees / 360 * ticks per turret revolution
        return degrees / 360 * kTicksPerRevolution / ratio;
    }

    public void resetPosition() {
        motor.setSelectedSensorPosition(0);
    }

    public void toggle() {
        isAutoEnabled = !isAutoEnabled;
    }

    public void panTurret() {
        if (this.getDegrees() >= kThresholdDegrees)
            panPower = -Math.abs(panPower);
        else if (this.getDegrees() <= -kThresholdDegrees)
            panPower = Math.abs(panPower);
        motor.set(ControlMode.PercentOutput, panPower);
    }

    @Log(name = "turret power")
    public double getPower(double setpoint) {
        return pid.calculate(this.getEncoderPosition(), setpoint);
    }

    public void adjust(boolean isTarget, double degrees) {
        if (!isAutoEnabled) {
            motor.set(ControlMode.PercentOutput, 0);
            return;
        }

        if (!isTarget) {
            this.panTurret();
            return;
        }

        double desiredDegrees = this.getDegrees() + degrees;

        if (desiredDegrees > kThresholdDegrees)
            desiredDegrees = kThresholdDegrees;
        else if (desiredDegrees < -kThresholdDegrees)
            desiredDegrees = -kThresholdDegrees;

        // motor.set(ControlMode.Position, this.convertDegreesToTicks(desiredDegrees));

        motor.set(ControlMode.PercentOutput, this.getPower(this.convertDegreesToTicks(degrees)));
    }

    public void set(double power) {
        isAutoEnabled = false;
        motor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("driver override", !isAutoEnabled);
        // SmartDashboard.putNumber("current degrees", this.getDegrees());
        // SmartDashboard.putNumber("turret encoder", this.getEncoderPosition());
    }
}
