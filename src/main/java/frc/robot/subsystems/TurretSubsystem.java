package frc.robot.subsystems;

import static frc.robot.Constants.Turret.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    // private final CANSparkMax motor = new CANSparkMax(6, MotorType.kBrushed);
    private final DigitalInput leftSwitch = new DigitalInput(0);
    private final DigitalInput rightSwitch = new DigitalInput(1);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    private final PIDController pid = new PIDController(kP, kI, kD);

    private boolean canPan = false;
    private double panPower = 0.5;
    // public TurretSubsystem() {

    // }

    public double getVelocityRPS() {
        return 0;
        // return motor.getEncoder().getVelocity() / 60; // rpm / 60 = rps
    }

    public void set(double power, boolean isAuto) {
        SmartDashboard.putNumber("panPower", panPower);

        if (!canPan && isAuto) {
            // motor.set(0);
            return;
        }

        if (power == 2 && canPan) { // if panning
            // if touching particular switch, go in opposite direction
            panPower = !leftSwitch.get() ? 0.5 : !rightSwitch.get() ? -0.5 : panPower;
            // set power to panPower
            power = panPower;
        } else if ((!leftSwitch.get() && power < 0) || (!rightSwitch.get() && power > 0)) { // else if touching switch
            // set power to 0
            power = 0;
        }

        double desiredRPS = power * maxRPS;
        // motor.setVoltage(feedforward.calculate(desiredRPS) + pid.calculate(this.getVelocityRPS(), desiredRPS));
    }

    public void toggle() {
        canPan = !canPan;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Turret can pan?", canPan);
    }
}
