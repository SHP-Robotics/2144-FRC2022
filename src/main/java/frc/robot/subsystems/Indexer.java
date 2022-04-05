package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Indexer.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final TalonSRX intake = new TalonSRX(7);
    private final TalonFX indexMotor = new TalonFX(8);
    private final TalonSRX guide = new TalonSRX(10);
    private final TalonSRX winch = new TalonSRX(9);

    private final DigitalInput firstSwitch = new DigitalInput(0);
    private final DigitalInput secondSwitch = new DigitalInput(2);
    // private final DigitalInput intakeSwitch = new DigitalInput(2);

    private boolean ballMoving = false;

    public Indexer() {
        indexMotor.configVoltageCompSaturation(kVoltageSaturation);
        indexMotor.configVoltageMeasurementFilter(kVoltageMeasurementSamples);
        indexMotor.enableVoltageCompensation(true);
        indexMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                true,
                kFalconContinuousCurrentLimit,
                kFalconPeakCurrentLimit,
                kFalconPeakCurrentDuration));
        indexMotor.setNeutralMode(NeutralMode.Brake);

        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
        talonConfig.continuousCurrentLimit = kTalonContinuousCurrentLimit;
        talonConfig.peakCurrentLimit = kTalonPeakCurrentLimit;
        talonConfig.peakCurrentDuration = kTalonPeakCurrentDuration;
        talonConfig.openloopRamp = kRampSeconds;

        intake.configAllSettings(talonConfig);

        guide.configAllSettings(talonConfig);
        guide.setInverted(true);

        // prime status frames to avoid overlapping frames and lower spikes
        winch.setStatusFramePeriod(StatusFrame.Status_1_General, 99);
        winch.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 101);
        // guide.follow(intake);
    }

    public void setIndexer(double speed) {
        indexMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setGuide(double speed) {
        guide.set(ControlMode.PercentOutput, speed);
    }

    public void setIntake(double speed) {
        intake.set(ControlMode.PercentOutput, speed);
    }

    public void setWinch(double speed) {
        winch.set(ControlMode.PercentOutput, speed);
    }

    public void intake() {
        if (this.ballIndexedFirst() && this.ballIndexedSecond())
            this.stopIntake();
        else
            this.setIntake(kIntakeSpeed);
    }

    public void outtake() {
        this.setIntake(-kIntakeSpeed);
    }

    public void stopIntake() {
        intake.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public boolean ballIndexedFirst() {
        return !firstSwitch.get();
        // return false;
    }

    public boolean ballIndexedSecond() {
        // return false;
        return !secondSwitch.get();
    }

    public boolean twoBallsIndexed() {
        return this.ballIndexedFirst() && this.ballIndexedSecond();
    }

    public int getNumBallsIndexed() {
        int numBallsIndexed = this.ballIndexedFirst() ? 1 : 0;
        if (this.ballIndexedSecond())
            numBallsIndexed += 1;
        return numBallsIndexed;
    }

    // public boolean isBallIntaked() {
    // return !intakeSwitch.get();
    // }

    @Override
    public void periodic() {
        // if a ball is found in the first index and a ball is moving
        if (this.ballIndexedFirst() && ballMoving) {
            // stop moving ball
            this.setGuide(0);
            this.setIndexer(0);
            ballMoving = false;
        }

        // if no ball in first index and ball in second index and indexer is not moving
        if (!this.ballIndexedFirst() && this.ballIndexedSecond() && indexMotor.getSelectedSensorVelocity() == 0) {
            // move ball into first index
            ballMoving = true;
        }

        if (ballMoving) {
            this.setGuide(kIntakeSpeed);
            this.setIndexer(kIndexerSpeed);
        }

        SmartDashboard.putNumber("index velocity", indexMotor.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("winch encoder", winch.getSelectedSensorPosition());
        SmartDashboard.putBoolean("first index", ballIndexedFirst());
        SmartDashboard.putBoolean("second index", ballIndexedSecond());

        // boolean ballIndexedFirst = isBallIndexedFirst();
        // boolean ballIndexedSecond = isBallindexedSecond();
        // boolean ballIntaked = isBallIntaked();
        // SmartDashboard.putBoolean("is ball indexed?", ballIndexedFirst);

        // // ballMoving ? moveBallToSecondIndex : stopMovingBall

        // // if no ball in first index and ball in second index
        // if (!ballIndexedFirst && ballIndexedSecond) {
        // // move ball into first index
        // }

        // // if a ball is found in second index and a ball is supposed to be moving
        // if (ballIndexedSecond && ballMoving) {
        // // stop moving ball
        // ballMoving = false;
        // }

        // if (ballIntaked) {
        // // if two balls indexed
        // if (ballIndexedFirst && ballIndexedSecond) {
        // // send ball back
        // } else {
        // // send ball forward
        // ballMoving = true;
        // }
        // }
    }
}
