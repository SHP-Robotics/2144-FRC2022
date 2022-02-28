package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Flywheel.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
    private final WPI_TalonFX leftLaunch = new WPI_TalonFX(4);
    private final WPI_TalonFX rightLaunch = new WPI_TalonFX(5);

    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
    private final BangBangController bangBangController = new BangBangController();

    private double desiredVelocityRPS;

    /**
     * Creates a new Flywheel. Controlled with a feedforward and a bang-bang
     * controlller.
     */
    public FlywheelSubsystem() {
        TalonFXConfiguration flywheelTalonConfig = new TalonFXConfiguration();
        flywheelTalonConfig.supplyCurrLimit.currentLimit = kFlywheelTalonCurrentLimit;
        flywheelTalonConfig.supplyCurrLimit.enable = true;
        flywheelTalonConfig.openloopRamp = 0.2;

        leftLaunch.configAllSettings(flywheelTalonConfig);
        rightLaunch.configAllSettings(flywheelTalonConfig);
        leftLaunch.setNeutralMode(NeutralMode.Coast);
        rightLaunch.setNeutralMode(NeutralMode.Coast);

        leftLaunch.setInverted(true);
        rightLaunch.setInverted(false);
    }

    /** @return the current falcon-reported velocity in rotations per second. */
    public double getTalonVelocityRPS() {
        // units/100ms * 10 = units/second
        // units/second * rotations/units = rotations/second
        return leftLaunch.getSelectedSensorVelocity() * 10 * kFlywheelRotationsPerPulse;
    }

    /**
     * @param rps
     * @return the velocity in meters per second
     */
    public double calculateVelocityMeters(double rps) {
        return Math.PI * kFlywheelDiameterMeters * rps; // circumference * rotations per second
    }

    /**
     * Sets the velocity setpoint for the flywheel.
     *
     * @param rotationsPerSecond velocity setpoint
     */
    public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
        desiredVelocityRPS = rotationsPerSecond;
    }

    /**
     * Applies the given voltage to the falcons.
     *
     * @param voltage what voltage to apply
     */
    public void setVoltage(double voltage) {
        leftLaunch.setVoltage(voltage);
        rightLaunch.setVoltage(voltage);
    }

    public void set(double speed) {
        leftLaunch.set(speed);
        rightLaunch.set(speed);
    }

    public boolean isAtSetpoint() {
        return this.getTalonVelocityRPS() >= desiredVelocityRPS;
    }

    @Override
    public void periodic() {
        // Calculates voltage to apply.
        // Feedforward is scaled down to prevent overshoot since bang-bang can't
        // correct for overshoot.

        // if trying to go backwards
        if (desiredVelocityRPS < 0) {
            this.set(-0.1);
            return;
        }

        double velocityRPS = this.getTalonVelocityRPS();

        // make motor stop if reversing before returning to ff + bb
        if (velocityRPS < 0) {
            this.set(0);
            return;
        }

        this.setVoltage(0.9 * flywheelFeedforward.calculate(desiredVelocityRPS)
                + bangBangController.calculate(velocityRPS, desiredVelocityRPS)
                        * kNominalVoltage);

        SmartDashboard.putNumber("velocity (m/s)", this.calculateVelocityMeters(velocityRPS));
        SmartDashboard.putNumber("current rps", velocityRPS);
        SmartDashboard.putNumber("desired rps", desiredVelocityRPS);
        // SmartDashboard.putNumber("left falcon applied voltage", leftLaunch.getBusVoltage());
        SmartDashboard.putNumber("Left Launch Temperature (°F)", 1.8 * leftLaunch.getTemperature() + 32);
        SmartDashboard.putNumber("Right Launch Temperature (°F)", 1.8 * rightLaunch.getTemperature() + 32);
    }
}
