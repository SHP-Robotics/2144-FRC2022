package frc.robot;

import static frc.robot.RobotContainer.*;

import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.utils.InterpolatingTreeMap;

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
    public static final double kFalconTicksPerRevolution = 2048;

    public static final int kFalconContinuousCurrentLimit = 40;
    public static final int kFalconPeakCurrentLimit = 60;
    public static final double kFalconPeakCurrentDuration = 0.1;

    public static final int kTalonContinuousCurrentLimit = 30;
    public static final int kTalonPeakCurrentLimit = 40;
    public static final int kTalonPeakCurrentDuration = 100;

    public static final class Control {
        public static JoystickButton kAButton = new JoystickButton(controller, 1);
        public static JoystickButton kBButton = new JoystickButton(controller, 2);
        public static JoystickButton kXButton = new JoystickButton(controller, 3);
        public static JoystickButton kYButton = new JoystickButton(controller, 4);

        public static JoystickButton kLBumper = new JoystickButton(controller, 5);
        public static JoystickButton kRBumper = new JoystickButton(controller, 6);

        public static JoystickButton kSelectButton = new JoystickButton(controller, 7);
        public static JoystickButton kStartButton = new JoystickButton(controller, 8);

        public static JoystickButton kLeftThumbPush = new JoystickButton(controller, 9);
        public static JoystickButton kRightThumbPush = new JoystickButton(controller, 10);

        public static POVButton kDpadUp = new POVButton(controller, 0);
        public static POVButton kDpadRight = new POVButton(controller, 90);
        public static POVButton kDpadDown = new POVButton(controller, 180);
        public static POVButton kDpadLeft = new POVButton(controller, 270);
    }

    public static final class Drive {
        public static final double kCollisionThresholdDeltaG = 0.5;

        public static final double kForwardThreshold = 0.6;
        public static final double kTurnThreshold = 0.4;
        public static final double kTurningSensitivity = 1;

        public static final double kTrackWidthInches = 23;
        public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);

        public static final double kWheelDiameterInches = 6;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final double kGearRatio = 10.75;

        public static final double kMetersPerTick = kWheelCircumferenceMeters * kGearRatio / kFalconTicksPerRevolution;
        public static final double kVelMetersToEnc = kFalconTicksPerRevolution / kGearRatio / kWheelCircumferenceMeters / 10;   
        public static final double kMomentOfInertia = 7.5; //kgm^2
        public static final double kMassKg = 100;
        public static final double kSpeedThreshold = 0.1; // m/s

        // imposed limits (not theoretical maximums)
        public static final double kMaxVelocity = 1; // m/s
        public static final double kMaxAcceleration = 1; // m/s^2

        // feedforward gains
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
    }

    public static final class Flywheel {
        // public static final double kVoltageSaturation = 12;
        // public static final int kVoltageMeasurementSamples = 32;

        public static final SensorVelocityMeasPeriod kVelocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_50Ms;
        public static final int kVelocityMeasurementWindow = 32;

        public static final double kMaxRPS = 30;
        public static final double kDefaultRPS = 10;
        public static final double kRPSDeadzone = 5; // need to change

        public static final double kRotationsPerTick = 1.0 / kFalconTicksPerRevolution;

        public static final double kRampSeconds = 0.1;

        // 1.5 : 1.0
        // public static final double kS = 0.59083;
        // public static final double kV = 0.50399;
        // public static final double kA = 0.013781;

        // 1.0 : 2.5
        // public static final double kS = 0.65454;
        // public static final double kV = 0.13604;
        // public static final double kA = 0.023556;

        public static final double kS = 0.58276;
        public static final double kV = 0.13622;
        public static final double kA = 0.025538;

        // public static final double kVelocityTolerance = 0.5;

        public static final double kFlywheelDiameterMeters = 0.102;

        public static final class Interpolation {
            public static final InterpolatingTreeMap<Number, Number> table = new InterpolatingTreeMap<>();

            static {
                // Distance (Inches) : Rotations Per Second
                table.put(0, 0);
            }
        }
    }

    public static final class Vision {
        public static final double kDeadzone = 5; // margin of acceptance for error

        public static final double kMountAngleDegrees = 0; // need to find (degrees rotated on mount)

        public static final double kCameraHeightInches = 0; // need to find (center of lens to floor)
        public static final double kTargetHeightInches = 104; // need to find (target to floor)
        public static final double kHeightDifference = kTargetHeightInches - kCameraHeightInches;

        public static final int kPipeline = 0;
    }

    public static final class Turret {
        // pid gains
        public static final double kP = 0.3; // 0.3
        public static final double kI = 0;// 0.0002;
        public static final double kD = 10;

        public static final double kCruiseVelocity = 6000;
        public static final double kAcceleration = 6000;

        public static final double kThresholdDegrees = 70; // for now

        public static final double ratio = kFalconTicksPerRevolution / 27400; // input : output

    }

    public static final class Indexer {
        public static final double kRampSeconds = 0.1;

        public static final double kIntakeSpeed = 0.8;
        public static final double kIndexerSpeed = 0.4;

        public static final double kVoltageSaturation = 12;
        public static final int kVoltageMeasurementSamples = 32;
    }

    public static final class Indicator {
        public static final double kRumbleValue = 0.5;
    }
}
