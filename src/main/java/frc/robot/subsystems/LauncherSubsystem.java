package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
    public TalonFX leftLaunch = new TalonFX(4);
    public TalonFX rightLaunch = new TalonFX(5);

    public LauncherSubsystem() {
        leftLaunch.setInverted(true);
        rightLaunch.setInverted(false);
    }

    public void set(double power) {
        leftLaunch.set(TalonFXControlMode.PercentOutput, power);
        rightLaunch.set(TalonFXControlMode.PercentOutput, power);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
}
