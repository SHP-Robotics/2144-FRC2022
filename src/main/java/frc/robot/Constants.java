// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kNominalVoltage = 12.0;

    public static final class Control {
        public static final int kAButton = 1;
        public static final int kBButton = 2;
        public static final int kXButton = 3;
        public static final int kYButton = 4;
        public static final int kLBumper = 5;
        public static final int kRBumper = 6;
        public static final int kSelect = 7;
        public static final int kStart = 8;
        public static final int kLeftThumbPush = 9;
        public static final int kRightThumbPush = 10;
    }

    public static final class Drive {
        public static final double kForwardThreshold = 0.6;
        public static final double kTurnThreshold = 0.5;

        public static final double kTurningSensitivity = 1.5;
    }

    public static final class Flywheel {
        public static final int kContinuousCurrentLimit = 40;
        public static final int kPeakCurrentLimit = 60;
        public static final double kPeakCurrentDuration = 0.1;

        public static final double kVoltageSaturation = 12;
        public static final int kVoltageMeasurementSamples = 32;

        public static final double kMaxRPS = 50;

        public static final int kTicksPerRevolution = 2048;
        public static final double kRotationsPerTick = 1.0 / kTicksPerRevolution;

        // 1.5 : 1.0
        // public static final double kS = 0.59083;
        // public static final double kV = 0.50399;
        // public static final double kA = 0.013781;

        // 1.0 : 2.5
        public static final double kS = 0.65454;
        public static final double kV = 0.13604;
        public static final double kA = 0.023556;

        // public static final double kVelocityTolerance = 0.5;

        public static final double kFlywheelDiameterMeters = 0.102;
    }

    public static final class Turret {
        // pid gains
        public static final double kP = 0.3; // 0.3
        public static final double kI = 0;// 0.0002;
        public static final double kD = 10;

        public static final double kTicksPerRevolution = 2048;
        public static final double kThresholdDegrees = 30; // for now

        public static final double ratio = kTicksPerRevolution / 27400; // input : output

    }
}
