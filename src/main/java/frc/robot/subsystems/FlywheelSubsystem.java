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
    private final WPI_TalonFX m_talon = new WPI_TalonFX(5);

    private final SimpleMotorFeedforward m_flywheelFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
    private final BangBangController m_bangBangController = new BangBangController();

    private double m_desiredVelocityRPS;

    /**
     * Creates a new Flywheel. Controlled with a feedforward and a bang bang
     * controlller.
     */
    public FlywheelSubsystem() {
        TalonFXConfiguration flywheelTalonConfig = new TalonFXConfiguration();
        flywheelTalonConfig.supplyCurrLimit.currentLimit = kFlywheelTalonCurrentLimit;
        flywheelTalonConfig.supplyCurrLimit.enable = true;
        flywheelTalonConfig.openloopRamp = 0.2;
        m_talon.configAllSettings(flywheelTalonConfig);

        m_talon.setNeutralMode(NeutralMode.Coast);
    }

    /** @return the current talon-reported velocity in rotations per second. */
    public double getTalonVelocity() {
        return m_talon.getSelectedSensorVelocity() * kFlywheelRotationsPerPulse * 10;
    }

    /**
     * Sets the velocity setpoint for the flywheel.
     *
     * @param rotationsPerSecond velocity setpoint
     */
    public void setVelocityRotationsPerSecond(double rotationsPerSecond) {
        m_desiredVelocityRPS = rotationsPerSecond;
    }

    /**
     * Applys the given voltage to the talon.
     *
     * @param voltage what voltage to apply
     */
    public void setVoltage(double voltage) {
        m_talon.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        // Calculates voltage to apply.
        // Feedforward is scaled down to prevent overshoot since bang-bang can't
        // correct
        // for overshoot.
        double velocity = getTalonVelocity();
        SmartDashboard.putNumber("talon velocity", velocity);
        SmartDashboard.putNumber("desired velocity", m_desiredVelocityRPS);
        
        if (m_desiredVelocityRPS < 0) {
            m_talon.set(-0.2);
            return;
        }

        double voltage = 0.95 * m_flywheelFeedforward.calculate(m_desiredVelocityRPS)
                + m_bangBangController.calculate(velocity, m_desiredVelocityRPS)
                        * kNominalVoltage;
        m_talon.setVoltage(voltage);

        SmartDashboard.putNumber("ff applied voltage", voltage);
        SmartDashboard.putNumber("talon applied voltage", m_talon.getBusVoltage());
    }
}