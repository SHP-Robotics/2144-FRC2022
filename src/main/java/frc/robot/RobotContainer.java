// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.MusicSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.TestSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
  // private final PneumaticSubsystem pneumaticSubsystem = new
  // PneumaticSubsystem();
  // private final TestSubsystem testSubsystem = new TestSubsystem();
  // private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  // private final MusicSubsystem musicSubsystem = new MusicSubsystem(new
  // TalonFX(0));

  // Commands
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);

  // Controllers
  public final XboxController driver = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Drive Command
    driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> driveSubsystem.drive(driver.getLeftY(),
                driver.getRightY(), -driver.getRightX()),
            driveSubsystem));

    flywheelSubsystem.setDefaultCommand(
        new RunCommand(
            () -> flywheelSubsystem.setVelocityRotationsPerSecond(
                (driver.getRightTriggerAxis() - driver.getLeftTriggerAxis())
                    * Constants.FlywheelConstants.kFlywheelMaxSpeedRotationsPerSecond),
            flywheelSubsystem));

    // testSubsystem.setDefaultCommand(new RunCommand(() -> {
    // testSubsystem.set(driver.getRightTriggerAxis() -
    // driver.getLeftTriggerAxis());
    // }, testSubsystem));

    // launcherSubsystem.setDefaultCommand(new RunCommand(() -> {
    // launcherSubsystem.set((driver.getRightTriggerAxis() -
    // driver.getLeftTriggerAxis()) * 0.8);
    // }, launcherSubsystem));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driver, Constants.Control.kAButton)
        .whenPressed(new InstantCommand(
            () -> driveSubsystem.switchMode(), driveSubsystem));

    // new JoystickButton(driver, Constants.Control.kXButton)
    // .whenPressed(new InstantCommand(
    // () -> pneumaticSubsystem.forward(), pneumaticSubsystem));

    // new JoystickButton(driver, Constants.Control.kYButton)
    // .whenPressed(new InstantCommand(
    // () -> pneumaticSubsystem.reverse(), pneumaticSubsystem));

    // new JoystickButton(driver, Constants.Control.kBButton)
    // .whenPressed(new InstantCommand(
    // () -> pneumaticSubsystem.off(), pneumaticSubsystem));

    // new JoystickButton(driver, Constants.Control.kBButton)
    // .whenPressed(new InstantCommand(
    // () -> musicSubsystem.play(), musicSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An ExampleCommand will run in autonomous
  // return m_autoCommand;
  // }
}
