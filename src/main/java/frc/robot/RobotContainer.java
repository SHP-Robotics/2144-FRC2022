package frc.robot;

import static frc.robot.Constants.Control.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.CycleBall;
import frc.robot.commands.FollowTrajectoryForward;
import frc.robot.commands.ForwardTimeBased;
import frc.robot.commands.ShootBall;
import frc.robot.commands.SpinFlywheel;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indicator;
import frc.robot.subsystems.MotorTest;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LightMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private final Drive drive = new Drive();
    private final Flywheel flywheel = new Flywheel();
    private final Vision vision = new Vision();
    public final Turret turret = new Turret();
    private final Indexer indexer = new Indexer();
    private final Indicator indicator = new Indicator();

    // private final MotorTest motorTest = new MotorTest();

    // private final PneumaticSubsystem pneumaticSubsystem = new
    // PneumaticSubsystem();
    // private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();

    // Commands
    // private final ExampleCommand m_autoCommand = new
    // ExampleCommand(m_exampleSubsystem);

    // Controllers
    public static final XboxController controller = new XboxController(0);

    // use flywheel is ready trigger to tell indexer to move
    private final Trigger ballIndexed = new Trigger(indexer::ballIndexedFirst);
    // private final Trigger targetLocked = new Trigger(vision::isTargetLocked);
    // private final Trigger robotStopped = new Trigger(drive::isStopped);
    // private final Trigger driverOverrideDisabled = new
    // Trigger(turret::isClosedLoop);

    /**
     * 
     * TODO
     * 
     * - Interpolation table
     * - Stop command (removes all scheduled comamnds and sets all motors to 0)
     * - Move controller IDs to Constants
     * - Move instantiations to constructor
     * 
     */

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        /**
         * DEFAULT COMMANDS
         */

        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.openLoop(controller.getLeftY(), -controller.getRightX()),
                        drive));

        indicator.setDefaultCommand(
                new RunCommand(
                        () -> indicator.update(indexer.getNumBallsIndexed(),
                                true, // vision.isTargetLocked(),
                                /* false), */ !turret.isClosedLoop()),
                        indicator));// , indexer, vision, turret));

        // flywheel.setDefaultCommand(
        // new RunCommand(
        // () -> flywheel.setDesiredVelocityRPS(
        // (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis())
        // * Constants.Flywheel.kMaxRPS),
        // flywheel));

        indexer.setDefaultCommand(
                new RunCommand(
                        () -> {
                            // indexer.setIndexer(0);
                            indexer.setGuide(0);
                            indexer.setIntake(0);
                            indexer.setWinch(controller.getRightTriggerAxis()
                                    - controller.getLeftTriggerAxis());
                        }, indexer));

        turret.setDefaultCommand(
                new RunCommand(
                        () -> turret.adjust(vision.isTarget(),
                                vision.getTx()),
                        turret));

        // launcherSubsystem.setDefaultCommand(new RunCommand(() -> {
        // launcherSubsystem.set((driver.getRightTriggerAxis() -
        // driver.getLeftTriggerAxis()) * 0.8);
        // }, launcherSubsystem));

        // indexer.setDefaultCommand(new RunCommand(() -> indexer.stopIntake(),
        // indexer));

        // exampleSubsystem.setDefaultCommand(
        // new RunCommand(
        // () -> exampleSubsystem.set(driver.getRightTriggerAxis() -
        // driver.getLeftTriggerAxis()),
        // exampleSubsystem));

        // Configure the button and trigger bindings
        configureButtonBindings();
        configureTriggerBindings();
    }

    /**
     * Use this method to define your button -> command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        /**
         * need to add stop command here (removes all scheduled commands and sets all
         * motors to 0)
         */

        // shoot ball
        // kAButton.debounce(0.1, DebounceType.kBoth)
        kAButton.and(ballIndexed)
                // .and(targetLocked)
                .whenActive(new CycleBall(flywheel, vision, indexer), false);

        // // turn on limelight LEDs
        kStartButton.whenPressed(new InstantCommand(
                () -> vision.setLedMode(LightMode.eOn), vision));

        // turn off limelight LEDs
        kSelectButton.whenPressed(new InstantCommand(
                () -> vision.setLedMode(LightMode.eOff), vision));

        // intake
        kRBumper.whileHeld(new InstantCommand(
                () -> {
                    // indexer.setIndexer(0.4);
                    indexer.setGuide(0.8);
                    indexer.intake();
                }, indexer));

        // outtake
        kLBumper.whileHeld(new InstantCommand(
                () -> {
                    // indexer.setIndexer(-0.4);
                    indexer.setGuide(-0.8);
                    indexer.outtake();
                }, indexer));

        // move turret right
        kDpadRight.whileHeld(new InstantCommand(
                () -> turret.openLoop(0.1), turret));

        // move turret left
        kDpadLeft.whileHeld(new InstantCommand(
                () -> turret.openLoop(-0.1), turret));

        // reset turret position
        kXButton.whenPressed(new InstantCommand(
                () -> turret.resetPosition(), turret));

        // toggle turret control
        kBButton.whenPressed(new InstantCommand(
                () -> turret.toggleLoop(), turret));

        // increase desired rps by 1
        kDpadUp.whileHeld(new InstantCommand(
                () -> flywheel.changeDesiredVelocityRPS(1), flywheel));

        // decrease desired rps by 1
        kDpadDown.whileHeld(new InstantCommand(
                () -> flywheel.changeDesiredVelocityRPS(-1), flywheel));

        // set desired rps to 0
        kYButton.whenPressed(new InstantCommand(
                () -> flywheel.setDesiredVelocityRPS(0), flywheel));

        // move indexer forward
        // kDpadUp.whenPressed(new InstantCommand(
        // () -> indexer.setIndexer(0.1), indexer));

        // // move indexer backward
        // kDpadDown.whenPressed(new InstantCommand(
        // () -> indexer.setIndexer(-0.1), indexer));

        // // stop moving indexer
        // kYButton.whenPressed(new InstantCommand(
        // () -> indexer.setIndexer(0), indexer));
    }

    public void configureTriggerBindings() {

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // // An ExampleCommand will run in autonomous
        // return new FollowTrajectoryForward(drive);
        return null;
    }
}
