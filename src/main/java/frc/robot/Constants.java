package frc.robot;

import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
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
    public static final double kTalonTicksPerRevolution = 2048;

    public static final class Control {
        public static JoystickButton kAButton;
        public static JoystickButton kBButton;
        public static JoystickButton kXButton;
        public static JoystickButton kYButton;

        public static JoystickButton kLBumper;
        public static JoystickButton kRBumper;

        public static JoystickButton kSelectButton;
        public static JoystickButton kStartButton;

        public static JoystickButton kLeftThumbPush;
        public static JoystickButton kRightThumbPush;

        public static POVButton kDpadUp;
        public static POVButton kDpadRight;
        public static POVButton kDpadDown;
        public static POVButton kDpadLeft;

        public static final void initButtons(XboxController controller) {
            kAButton = new JoystickButton(controller, 1);
            kBButton = new JoystickButton(controller, 2);
            kXButton = new JoystickButton(controller, 3);
            kYButton = new JoystickButton(controller, 4);

            kLBumper = new JoystickButton(controller, 5);
            kRBumper = new JoystickButton(controller, 6);

            kSelectButton = new JoystickButton(controller, 7);
            kStartButton = new JoystickButton(controller, 8);

            kLeftThumbPush = new JoystickButton(controller, 9);
            kRightThumbPush = new JoystickButton(controller, 10);

            kDpadUp = new POVButton(controller, 0);
            kDpadRight = new POVButton(controller, 90);
            kDpadDown = new POVButton(controller, 180);
            kDpadLeft = new POVButton(controller, 270);
        }
    }

    public static final class Drive {
        public static final double kCollisionThresholdDeltaG = 0.5;

        public static final double kForwardThreshold = 0.6;
        public static final double kTurnThreshold = 0.4;
        public static final double kTurningSensitivity = 1.5;

        public static final double kTrackWidthInches = 23;
        public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);

        public static final double kWheelDiameterMeters = 0; // need to get
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final double kMetersPerTick = kWheelCircumferenceMeters / kTalonTicksPerRevolution;

        // imposed limits (not theoretical maximums)
        public static final double kMaxVelocity = 1; // m/s
        public static final double kMaxAcceleration = 1; // m/s^2

        // feedforward gains
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
    }

    public static final class Flywheel {
        public static final int kContinuousCurrentLimit = 40;
        public static final int kPeakCurrentLimit = 60;
        public static final double kPeakCurrentDuration = 0.1;

        // public static final double kVoltageSaturation = 12;
        // public static final int kVoltageMeasurementSamples = 32;

        public static final SensorVelocityMeasPeriod kVelocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_50Ms;
        public static final int kVelocityMeasurementWindow = 32;

        public static final double kMaxRPS = 30;
        public static final double kDefaultRPS = 10;

        public static final double kRotationsPerTick = 1.0 / kTalonTicksPerRevolution;

        public static final double kRampSeconds = 0.2;

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
    }

    public static final class Turret {
        // pid gains
        public static final double kP = 0.3; // 0.3
        public static final double kI = 0;// 0.0002;
        public static final double kD = 10;

        public static final double kThresholdDegrees = 30; // for now

        public static final double ratio = kTalonTicksPerRevolution / 27400; // input : output

    }

    public static final class Indexer {
        public static final double kDefaultSpeed = 0.3;

        public static final double kVoltageSaturation = 12;
        public static final int kVoltageMeasurementSamples = 32;
    }
}
