package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Indexer.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
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
    private final TalonSRX winch = new TalonSRX(9);
    private final TalonSRX guide = new TalonSRX(10);

    private final DigitalInput firstSwitch = new DigitalInput(0);
    private final DigitalInput secondSwitch = new DigitalInput(2);
    // private final DigitalInput intakeSwitch = new DigitalInput(4);

    private boolean ballMoving = false;
    private boolean outtaking = false;

    /**
     * Creates a new Indexer containing the index, intake, guide, and winch motors.
     */
    public Indexer() {
        indexMotor.configVoltageCompSaturation(kVoltageSaturation);
        indexMotor.configVoltageMeasurementFilter(kVoltageMeasurementSamples);
        indexMotor.enableVoltageCompensation(true);
        indexMotor.configSupplyCurrentLimit(kFalconSupplyCurrentConfig);
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

    /**
     * Sets the open-loop speed for the indexer.
     * 
     * @param speed The speed to set, between 1.0 and -1.0.
     */
    public void setIndexer(double speed) {
        outtaking = speed < 0;
        indexMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the open-loop speed for the guide.
     * 
     * @param speed The speed to set, between 1.0 and -1.0.
     */
    public void setGuide(double speed) {
        guide.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the open-loop speed for the intake.
     * 
     * @param speed The speed to set, between 1.0 and -1.0.
     */
    public void setIntake(double speed) {
        intake.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the open-loop speed for the intake winch.
     * 
     * @param speed The speed to set, between 1.0 and -1.0.
     */
    public void setWinch(double speed) {
        winch.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Intakes balls unless both indices are full.
     */
    public void intake() {
        if (this.ballIndexedFirst() && this.ballIndexedSecond())
            this.stopIntake();
        else
            this.setIntake(kIntakeSpeed);
    }

    /**
     * Outtakes balls.
     */
    public void outtake() {
        this.setIntake(-kIntakeSpeed);
    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake() {
        intake.set(TalonSRXControlMode.PercentOutput, 0);
    }

    /**
     * @return Whether or not a ball is in the first index.
     */
    public boolean ballIndexedFirst() {
        return !firstSwitch.get();
        // return false;
    }

    /**
     * @return Whether or not a ball is in the second index.
     */
    public boolean ballIndexedSecond() {
        // return false;
        return !secondSwitch.get();
    }

    /**
     * @return Whether or not there are balls in both indices.
     */
    public boolean twoBallsIndexed() {
        return this.ballIndexedFirst() && this.ballIndexedSecond();
    }

    /**
     * @return The number of balls indexed.
     */
    public int getNumBallsIndexed() {
        int numBallsIndexed = this.ballIndexedFirst() ? 1 : 0;
        if (this.ballIndexedSecond())
            numBallsIndexed += 1;
        return numBallsIndexed;
    }

    @Override
    public void periodic() {
        // if a ball is found in the first index and a ball is moving
        if (this.ballIndexedFirst() && ballMoving) {
            // stop moving ball
            this.setGuide(0);
            this.setIndexer(0);
            ballMoving = false;
        }

        // if no ball in first index and ball in second index and not outtaking a ball
        if (!this.ballIndexedFirst() && this.ballIndexedSecond() && !outtaking) {
            // move ball into first index
            ballMoving = true;
        }

        // if ball is supposed to be moving, move it into first index
        if (ballMoving) {
            this.setGuide(kIntakeSpeed);
            this.setIndexer(kIndexerSpeed);
        }

        // SmartDashboard.putNumber("winch encoder", winch.getSelectedSensorPosition());

        SmartDashboard.putBoolean("First Index [0]", this.ballIndexedFirst());
        SmartDashboard.putBoolean("Second Index [1]", this.ballIndexedSecond());
    }
}
