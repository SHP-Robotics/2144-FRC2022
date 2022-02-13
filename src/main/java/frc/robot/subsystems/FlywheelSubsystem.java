package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.FlywheelConstants.*;

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
    public double getTalonVelocity() {
        return leftLaunch.getSelectedSensorVelocity() * kFlywheelRotationsPerPulse * 10;
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

    @Override
    public void periodic() {
        // Calculates voltage to apply.
        // Feedforward is scaled down to prevent overshoot since bang-bang can't
        // correct for overshoot.
        
        // if trying to go backwards
        if (desiredVelocityRPS < 0) {
            leftLaunch.set(-0.2);
            rightLaunch.set(-0.2);
            return;
        }

        double velocity = this.getTalonVelocity();

        // wait for motor to come to rest before returning to ff + bb
        if (velocity < 0) return;

        double voltage = 0.95 * flywheelFeedforward.calculate(desiredVelocityRPS)
                + bangBangController.calculate(velocity, desiredVelocityRPS)
                        * kNominalVoltage;
        this.setVoltage(voltage);

        SmartDashboard.putNumber("ff applied voltage", voltage);
        SmartDashboard.putNumber("left falcon applied voltage", leftLaunch.getBusVoltage());
        SmartDashboard.putNumber("Left Launch Temperature ('C)", leftLaunch.getTemperature());
        SmartDashboard.putNumber("Right Launch Temperature ('C)", rightLaunch.getTemperature());
    }
}
