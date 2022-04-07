// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.Math;

/**
 * Wrapper class for getting and setting Limelight NetworkTable values.
 * 
 * @author Dan Waxman
 */
public class Vision extends SubsystemBase {
	private final NetworkTableInstance table;

	public Vision() {
		table = NetworkTableInstance.getDefault();
		this.setPipeline(kPipeline);
	}

	/**
	 * Light modes for Limelight.
	 * 
	 * @author Dan Waxman
	 */
	public static enum LightMode {
		eOn, eOff, eBlink
	}

	/**
	 * Camera modes for Limelight.
	 * 
	 * @author Dan Waxman
	 */
	public static enum CameraMode {
		eVision, eDriver
	}

	/**
	 * Gets whether a target is detected by the Limelight.
	 * 
	 * @return true if a target is detected, false otherwise.
	 */
	public boolean isTarget() {
		return this.getValue("tv").getDouble(0) == 1;
	}

	/**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 * 
	 * @return tx as reported by the Limelight.
	 */
	public double getTx() {
		return this.getValue("tx").getDouble(0.00);
	}

	/**
	 * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
	 * 
	 * @return ty as reported by the Limelight.
	 */
	public double getTy() {
		return this.getValue("ty").getDouble(0.00);
	}

	/**
	 * Area that the detected target takes up in total camera FOV (0% to 100%).
	 * 
	 * @return Area of target.
	 */
	public double getTa() {
		return this.getValue("ta").getDouble(0.00);
	}

	/**
	 * Gets target skew or rotation (-90 degrees to 0 degrees).
	 * 
	 * @return Target skew.
	 */
	public double getTs() {
		return this.getValue("ts").getDouble(0.00);
	}

	/**
	 * Gets target latency (ms).
	 * 
	 * @return Target latency.
	 */
	public double getTl() {
		return this.getValue("tl").getDouble(0.00);
	}

	/**
	 * Sets LED mode of Limelight.
	 * 
	 * @param mode
	 *             Light mode for Limelight.
	 */
	public void setLedMode(LightMode mode) {
		this.getValue("ledMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets camera mode for Limelight.
	 * 
	 * @param mode Camera mode for Limelight.
	 */
	public void setCameraMode(CameraMode mode) {
		this.getValue("camMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets pipeline number (0-9 value).
	 * 
	 * @param number Pipeline number (0-9).
	 */
	public void setPipeline(int number) {
		this.getValue("pipeline").setNumber(number);
	}

	// public boolean isTargetLocked() {
	// double offset = getTx();
	// SmartDashboard.putNumber("offset degrees", offset);
	// return isTarget() && offset < kDeadzone && offset > -kDeadzone;
	// }

	/**
	 * Helper method to get an entry from the Limelight NetworkTable.
	 * 
	 * @param key Key for entry.
	 * @return NetworkTableEntry of given entry.
	 */
	private NetworkTableEntry getValue(String key) {
		return table.getTable("limelight").getEntry(key);
	}

	/**
	 * https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-a-fixed-angle-camera
	 * 
	 * @return The camera's distance from the target in inches.
	 */
	public double getDistanceInches() {
		double angleToGoalRadians = Math.toRadians(kMountAngleDegrees + this.getTy());
		return kHeightDifference / Math.tan(angleToGoalRadians);
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Target", isTarget());
		SmartDashboard.putNumber("Offset", getTx());
		SmartDashboard.putNumber("Distance Inches", getDistanceInches());
	}
}
