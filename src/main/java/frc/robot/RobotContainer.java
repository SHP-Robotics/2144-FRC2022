// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    // private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

    // private final Drive drive = new Drive();
    // private final Flywheel flywheel = new Flywheel();
    // private final Vision vision = new Vision();
    // private final Turret turret = new Turret();
    private final Intake intake = new Intake();

    // private final PneumaticSubsystem pneumaticSubsystem = new
    // PneumaticSubsystem();
    // private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();

    // Commands
    // private final ExampleCommand m_autoCommand = new
    // ExampleCommand(m_exampleSubsystem);

    // Controllers
    public final XboxController driver = new XboxController(0);

    // use flywheel is ready trigger to tell indexer to move
    // Trigger flywheelReady = new Trigger(flywheelSubsystem::isAtSetpoint);
    // Trigger ballIndexed = new Trigger(IndexingSubsystem::isBallIndexed);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // cnofigure logger
        Logger.configureLoggingAndConfig(this, false);

        // drive.setDefaultCommand(
        // new RunCommand(
        // () -> drive.drive(driver.getLeftY(),
        // driver.getRightY(), -driver.getRightX()),
        // drive));

        // flywheel.setDefaultCommand(
        //         new RunCommand(
        //                 () -> flywheel.setVelocityRotationsPerSecond(
        //                         (driver.getRightTriggerAxis() - driver.getLeftTriggerAxis())
        //                                 * Constants.Flywheel.kFlywheelMaxSpeedRotationsPerSecond),
        //                 flywheel));

        // turret.setDefaultCommand(
        // new RunCommand(
        // () -> turret.adjust(vision.isTarget(),
        // vision.getTx()),
        // turret));

        // flywheelReady.and(ballIndexed).whenActive(new StartEndCommand());

        // launcherSubsystem.setDefaultCommand(new RunCommand(() -> {
        // launcherSubsystem.set((driver.getRightTriggerAxis() -
        // driver.getLeftTriggerAxis()) * 0.8);
        // }, launcherSubsystem));

        intake.setDefaultCommand(new RunCommand(() -> {
        intake.set((driver.getRightTriggerAxis() -
        driver.getLeftTriggerAxis()) * 1.0);
        }, intake));

        // exampleSubsystem.setDefaultCommand(
        // new RunCommand(
        // () -> exampleSubsystem.set(driver.getRightTriggerAxis() -
        // driver.getLeftTriggerAxis()),
        // exampleSubsystem));

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
        // kill switch
        // new JoystickButton(driver, Constants.Control.kSelect)
        // .whenPressed(new InstantCommand(
        // () -> CommandScheduler.getInstance().disable()));

        // // enable switch
        // new JoystickButton(driver, Constants.Control.kStart)
        // .whenPressed(new InstantCommand(
        // () -> CommandScheduler.getInstance().enable()));

        // switch drive mode
        // new JoystickButton(driver, Constants.Control.kAButton)
        // .whenPressed(new InstantCommand(
        // () -> drive.switchMode(), drive));

        // // move turret right
        // new POVButton(driver, 90)
        // .whileHeld(new InstantCommand(
        // () -> turret.openLoop(0.1), turret));

        // // move turret left
        // new POVButton(driver, 270)
        // .whileHeld(new InstantCommand(
        // () -> turret.openLoop(-0.1), turret));

        // // reset turret position
        // new JoystickButton(driver, Constants.Control.kXButton)
        // .whenPressed(new InstantCommand(
        // () -> turret.resetPosition(), turret));

        // // toggle turret control
        // new JoystickButton(driver, Constants.Control.kBButton)
        // .whenPressed(new InstantCommand(
        // () -> turret.toggleLoop(), turret));

        // // test
        // new JoystickButton(driver, Constants.Control.kYButton)
        // .whenPressed(new InstantCommand(
        // () -> turret.adjust(true, 30), turret));

        // new JoystickButton(driver, Constants.Control.kXButton)
        // .whenPressed(new InstantCommand(
        // () -> pneumaticSubsystem.forward(), pneumaticSubsystem));

        // new JoystickButton(driver, Constants.Control.kYButton)
        // .whenPressed(new InstantCommand(
        // () -> pneumaticSubsystem.reverse(), pneumaticSubsystem));

        // new JoystickButton(driver, Constants.Control.kBButton)
        // .whenPressed(new InstantCommand(
        // () -> pneumaticSubsystem.off(), pneumaticSubsystem));
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
