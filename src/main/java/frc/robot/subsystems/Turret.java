package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Turret.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Turret extends SubsystemBase implements Loggable {
    private final WPI_TalonFX motor = new WPI_TalonFX(6);

    @Log(name = "auto enabled")
    private boolean closedLoop = false;
    private double panPower = 0.1;

    // double power = 0;

    // private final ProfiledPIDController pid = new ProfiledPIDController(kP, 0, 0,
    // new TrapezoidProfile.Constraints(1, 1));

    public Turret() {
        motor.config_kP(0, kP);
        motor.config_kI(0, kI);
        motor.config_kD(0, kD);

        // motor.config_kP(0, 0);
        // motor.config_kI(0, 0);
        // motor.config_kD(0, 0);

        motor.configMotionCruiseVelocity(5000);
        motor.configMotionAcceleration(5000);

        motor.setInverted(true);
    }

    @Log(name = "turret encoder")
    public double getEncoderPosition() {
        return motor.getSelectedSensorPosition();
    }

    @Log(name = "turret degrees")
    public double getDegrees() {
        // turret positon / ticks per revolution * 360
        return this.getEncoderPosition() * ratio / kTalonTicksPerRevolution * 360;
    }

    public double getRadians(double pulses) {
        return Math.toRadians(this.getDegrees());
    }

    public double convertDegreesToTicks(double degrees) {
        // degrees / 360 * ticks per turret revolution
        return degrees / 360 * kTalonTicksPerRevolution / ratio;
    }

    public void resetPosition() {
        motor.setSelectedSensorPosition(0);
    }

    public void toggleLoop() {
        closedLoop = !closedLoop;
    }

    public void panTurret() {
        if (this.getDegrees() >= kThresholdDegrees)
            panPower = -Math.abs(panPower);
        else if (this.getDegrees() <= -kThresholdDegrees)
            panPower = Math.abs(panPower);
        motor.set(ControlMode.PercentOutput, panPower);
    }

    // @Log(name = "turret power")
    // public double getPower(double setpoint) {
    // power = pid.calculate(this.getEncoderPosition(), setpoint);
    // return power;
    // }

    public void adjust(boolean isTarget, double degrees) {
        if (!closedLoop) {
            motor.set(ControlMode.PercentOutput, 0);
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

        motor.set(ControlMode.Position, this.convertDegreesToTicks(desiredDegrees));

        // this.getPower(this.convertDegreesToTicks(desiredDegrees));
    }

    public void openLoop(double power) {
        if (closedLoop)
            closedLoop = false;

        double degrees = this.getDegrees();
        if ((degrees > kThresholdDegrees && power > 0) || (degrees < -kThresholdDegrees && power < 0))
            return;

        motor.set(ControlMode.PercentOutput, power);
    }

    public boolean isClosedLoop() {
        return closedLoop;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("driver override", !closedLoop);
        SmartDashboard.putNumber("current degrees", this.getDegrees());
        SmartDashboard.putNumber("turret encoder", this.getEncoderPosition());
        // SmartDashboard.putNumber("turret power", power);
    }
}
