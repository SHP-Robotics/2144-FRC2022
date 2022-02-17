package frc.robot.subsystems;

import static frc.robot.Constants.Turret.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(6, MotorType.kBrushed);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    private final PIDController pid = new PIDController(kP, kI, kD);

    private boolean canPan = false;
    // public TurretSubsystem() {

    // }

    public double getVelocityRPS() {
        return motor.getEncoder().getVelocity() / 60; // rpm / 60 = rps
    }

    public void set(double power, boolean isAuto) {
        if (!canPan && isAuto) {
            motor.set(0);
            return;
        }
        double desiredRPS = power * maxRPS;
        motor.setVoltage(feedforward.calculate(desiredRPS) + pid.calculate(this.getVelocityRPS(), desiredRPS));
    }

    public void toggle() {
        canPan = !canPan;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Turret can pan?", canPan);
    }
}
