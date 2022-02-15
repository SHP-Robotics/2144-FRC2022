package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Turret.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(6, MotorType.kBrushless);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    private final PIDController pid = new PIDController(kP, kI, kD);

    public TurretSubsystem() {

    }

    public double getVelocityRPS() {
        return motor.getEncoder().getVelocity() / 60;
    }

    public void set(double power) {
        double desiredRPS = power * maxRPS;
        motor.setVoltage(feedforward.calculate(desiredRPS) + pid.calculate(this.getVelocityRPS(), desiredRPS));
    }

    // @Override
    // public void periodic() {
    // // This method will be called once per scheduler run
    // }
}