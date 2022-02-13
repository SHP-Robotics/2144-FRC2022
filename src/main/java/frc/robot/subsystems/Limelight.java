// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

/**
 * Wrapper class for getting and setting Limelight NetworkTable values.
 * 
 * @author Dan Waxman
 */
public class Limelight {
	private static NetworkTableInstance table = null;

	double reflectiveTapeLength = 5*2.54; //inches*2.54=cm
	double knownHeight = 10; //TODO: find value from top height of reflective tape - height of camera point AND convert to inches
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
	public static boolean isTarget() {
		return getValue("tv").getDouble(0) == 1;
	}

	/**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 * 
	 * @return tx as reported by the Limelight.
	 */
	public static double getTx() {
		return getValue("tx").getDouble(0.00);
	}

	/**
	 * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
	 * 
	 * @return ty as reported by the Limelight.
	 */
	public static double getTy() {
		return getValue("ty").getDouble(0.00);
	}

	/**
	 * Area that the detected target takes up in total camera FOV (0% to 100%).
	 * 
	 * @return Area of target.
	 */
	public static double getTa() {
		return getValue("ta").getDouble(0.00);
	}

	/**
	 * Gets target skew or rotation (-90 degrees to 0 degrees).
	 * 
	 * @return Target skew.
	 */
	public static double getTs() {
		return getValue("ts").getDouble(0.00);
	}

	/**
	 * Gets target latency (ms).
	 * 
	 * @return Target latency.
	 */
	public static double getTl() {
		return getValue("tl").getDouble(0.00);
	}

	/**
	 * Sets LED mode of Limelight.
	 * 
	 * @param mode
	 *            Light mode for Limelight.
	 */
	public static void setLedMode(LightMode mode) {
		getValue("ledMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets camera mode for Limelight.
	 * 
	 * @param mode
	 *            Camera mode for Limelight.
	 */
	public static void setCameraMode(CameraMode mode) {
		getValue("camMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets pipeline number (0-9 value).
	 * 
	 * @param number
	 *            Pipeline number (0-9).
	 */
	public static void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

	/**
	 * Helper method to get an entry from the Limelight NetworkTable.
	 * 
	 * @param key
	 *            Key for entry.
	 * @return NetworkTableEntry of given entry.
	 */
	private static NetworkTableEntry getValue(String key) {
		if (table == null) {
			table = NetworkTableInstance.getDefault();
		}

		return table.getTable("limelight").getEntry(key);
	}


	public static double[] getCorners(){
		double[] placeHoler = new double [8]; //temporary fix for returning corners. 
		return getValue("tcornxy").getDoubleArray(placeHoler); //returns array of corner cordinates x0, y0...
	}

	public static double pixelsToAngle(double pixelXValue){ 
		if(pixelXValue > 160){
			pixelXValue = pixelXValue-160;
		}
		 return pixelXValue/(320/59.6); //returns angle from center, center is 0 degrees 
	}

	public static double getHypotenuse(double triangulationLength){ //finds the hypotenuse
		double distance;
		double a, b; //angles for triangulation
		double[] corners = getCorners();

		a = 90 - pixelsToAngle(corners[0]); //hopefully these corners are the top left and right
		b = 90 - pixelsToAngle(corners[4]);


		distance = (triangulationLength*Math.sin(a)*Math.sin(b))/Math.sin(a+b);

		return distance;
	}

	public static double getAngle(double knownHeight){ // gets angle from ground (0 degrees) to angle from camera to top of reflective tape
		double sideC = getHypotenuse(5*2.54); //length of reflective tape
		
		return Math.asin(knownHeight/sideC);
	}

	public static double getDistance(double knownHeight){ //distance from camera to net thing with the tape 
		double sideC = getHypotenuse(5*2.54); //length of reflective tape

		return Math.sqrt((sideC*sideC)-(knownHeight*knownHeight));
	}


}