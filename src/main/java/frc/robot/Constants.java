// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    // public static final class Drive {
        // public static final CANSparkMax[] kMotors = {
    //         new CANSparkMax(1, MotorType.kBrushless), // Left Front
    //         new CANSparkMax(2, MotorType.kBrushless), // Left Back
    //         new CANSparkMax(3, MotorType.kBrushless), // Right Front
    //         new CANSparkMax(4, MotorType.kBrushless) // Right Back
    // };
    // public static final MotorControllerGroup leftDrive = new MotorControllerGroup(kMotors[0]);//, kMotors[1]);
    // public static final MotorControllerGroup rightDrive = new MotorControllerGroup(kMotors[2]);//, kMotors[3]);


    //     // public static final PWMSparkMax leftFront = new PWMSparkMax(0);
    //     // public static final PWMSparkMax leftBack = new PWMSparkMax(1);
    //     // public static final PWMSparkMax rightFront = new PWMSparkMax(2);
    //     // public static final PWMSparkMax rightBack = new PWMSparkMax(3);
    // }

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
}
