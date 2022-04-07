package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Flywheel.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final WPI_TalonFX leftLaunch = new WPI_TalonFX(4);
    private final WPI_TalonFX rightLaunch = new WPI_TalonFX(5);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    private final BangBangController bangbang = new BangBangController();

    private double desiredVelocityRPS;

    /**
     * Creates a new Flywheel controlled by a feedforward and bang-bang controller.
     */
    public Flywheel() {
        TalonFXConfiguration falconConfig = new TalonFXConfiguration();
        falconConfig.supplyCurrLimit = kFalconSupplyCurrentConfig;
        falconConfig.velocityMeasurementPeriod = kVelocityMeasurementPeriod;
        falconConfig.velocityMeasurementWindow = kVelocityMeasurementWindow;
        falconConfig.openloopRamp = kRampSeconds;

        leftLaunch.configAllSettings(falconConfig);
        rightLaunch.configAllSettings(falconConfig);
        leftLaunch.setNeutralMode(NeutralMode.Coast);
        rightLaunch.setNeutralMode(NeutralMode.Coast);
        leftLaunch.setInverted(false);
        rightLaunch.setInverted(true);
    }

    /**
     * Enables acceleration ramp.
     */
    public void enableRamp() {
        leftLaunch.configOpenloopRamp(kRampSeconds);
        rightLaunch.configOpenloopRamp(kRampSeconds);
    }

    /**
     * Disables acceleration ramp.
     */
    public void disableRamp() {
        leftLaunch.configOpenloopRamp(0);
        rightLaunch.configOpenloopRamp(0);
    }

    /**
     * @return The current falcon-reported velocity in rotations per second.
     */
    public double getTalonVelocityRPS() {
        // ticks/100ms * 10 = units/second
        // ticks/second * rotations/tick = rotations/second
        return leftLaunch.getSelectedSensorVelocity() * 10 * kRotationsPerTick;
    }

    /**
     * @param rps Rotations per second.
     * @return The current falcon-reported velocity in meters per second.
     */
    public double calculateVelocityMeters(double rps) {
        return Math.PI * kFlywheelDiameterMeters * rps; // circumference * rotations per second
    }

    /**
     * Changes the desired velocity setpoint by the given setpoint.
     * 
     * @param rps Rotations per second.
     */
    public void changeDesiredVelocityRPS(double rps) {
        desiredVelocityRPS += rps;
    }

    /**
     * Sets the velocity setpoint for the flywheel.
     *
     * @param rps Rotations per second.
     */
    public void setDesiredVelocityRPS(double rps) {
        desiredVelocityRPS = rps;
    }

    /**
     * Applies the given voltage to the falcons.
     *
     * @param voltage The voltage to apply.
     */
    public void setVoltage(double voltage) {
        leftLaunch.setVoltage(voltage);
        rightLaunch.setVoltage(voltage);
    }

    /**
     * Sets the open-loop speed for the flywheel.
     * 
     * @param speed The speed to set, between 1.0 and -1.0.
     */
    public void set(double speed) {
        leftLaunch.set(speed);
        rightLaunch.set(speed);
    }

    /**
     * @return Whether the flywheel is at the setpoint or not.
     */
    public boolean isAtSetpoint() {
        double velocityRPS = this.getTalonVelocityRPS();
        return velocityRPS >= desiredVelocityRPS && velocityRPS <= desiredVelocityRPS + kRPSDeadzone;
    }

    @Override
    public void periodic() {
        double velocityRPS = this.getTalonVelocityRPS();

        // if trying to go backwards
        if (desiredVelocityRPS < 0 && velocityRPS <= 0) {
            this.set(-0.1);
            return;
        }

        // make motor stop if reversing before returning to ff + bb
        if (velocityRPS < 0) {
            this.set(0);
            return;
        }

        this.setVoltage(0.95 * feedforward.calculate(desiredVelocityRPS)
                + bangbang.calculate(velocityRPS, desiredVelocityRPS)
                        * kNominalVoltage);

        SmartDashboard.putNumber("velocity (m/s)", this.calculateVelocityMeters(velocityRPS));
        SmartDashboard.putNumber("current rps", velocityRPS);
        SmartDashboard.putNumber("desired rps", desiredVelocityRPS);
        // SmartDashboard.putNumber("left falcon applied voltage",
        // leftLaunch.getBusVoltage());
        SmartDashboard.putNumber("Left Launch Temperature (°F)", 1.8 * leftLaunch.getTemperature() + 32);
        SmartDashboard.putNumber("Right Launch Temperature (°F)", 1.8 * rightLaunch.getTemperature() + 32);
    }
}
