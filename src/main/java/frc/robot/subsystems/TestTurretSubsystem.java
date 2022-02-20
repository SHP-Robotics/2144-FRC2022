package frc.robot.subsystems;

import static frc.robot.Constants.Turret.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestTurretSubsystem extends SubsystemBase {
    private final TalonSRX motor = new TalonSRX(6);

    // private final SimpleMotorFeedforward feedforward = new
    // SimpleMotorFeedforward(kS, kV, kA);
    // private final PIDController pid = new PIDController(kP, kI, kD);

    // public TurretSubsystem() {

    // }




    

    // add driver override (automatically disables auto panning when direction is
    // pressed)






    public double getEncoderPosition() {
        return motor.getSelectedSensorPosition();
    }

    public double getDegrees() {
        // encoder positon / ticks per revolution * 360
        return this.getEncoderPosition() / kTicksPerRevolution * 360;
    }

    public double getRadians(double pulses) {
        return Math.toRadians(this.getDegrees());
    }

    public double convertDegreesToTicks(double degrees) {
        // degrees / 360 * ticks per revolution
        return degrees / 360 * kTicksPerRevolution;
    }

    public void resetPosition() {
        motor.setSelectedSensorPosition(0);
    }

    public void moveByDegrees(double degrees) {
        double desiredDegrees = this.getDegrees() + degrees;

        if (desiredDegrees > kRightThresholdDegrees)
            desiredDegrees = kRightThresholdDegrees;
        else if (desiredDegrees < kLeftThresholdDegrees)
            desiredDegrees = kLeftThresholdDegrees;

        motor.set(ControlMode.Position, this.convertDegreesToTicks(degrees));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("current degrees", this.getDegrees());
    }
}
