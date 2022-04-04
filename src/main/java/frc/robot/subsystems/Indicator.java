package frc.robot.subsystems;

import static frc.robot.RobotContainer.*;
import static frc.robot.Constants.Indicator.*;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.BlinkinLedDriver;
import frc.robot.utils.BlinkinLedDriver.BlinkinLedMode;

public class Indicator extends SubsystemBase {
    // private final BlinkinLedDriver led = new BlinkinLedDriver(0);

    public void update(boolean ballIndexed, boolean targetLocked, boolean driverOverride) {
        // if () {
        //     startRumble();
        // //     // setBlue();
        // //     return;
        // }

        if ((ballIndexed && targetLocked) || (ballIndexed && driverOverride)) {
            startRumble();
            // setGreenBlinking();
            // return;
        } else
            stopRumble();

        // if (ballIndexed && !targetLocked) {
        //     // setGreenSolid();
        // } else if (!ballIndexed && targetLocked) {
        //     // setRedBlinking();
        // } //else
            // setRedSolid();

    }

    private void startRumble() {
        controller.setRumble(RumbleType.kLeftRumble, kRumbleValue);
        controller.setRumble(RumbleType.kRightRumble, kRumbleValue);
    }

    private void stopRumble() {
        controller.setRumble(RumbleType.kLeftRumble, 0);
        controller.setRumble(RumbleType.kRightRumble, 0);
    }

    // private void setBlue() {
    //     led.setMode(BlinkinLedMode.SOLID_BLUE);
    // }

    // private void setGreenBlinking() {
    //     led.setMode(BlinkinLedMode.ONE_STROBE);
    // }

    // private void setGreenSolid() {
    //     led.setMode(BlinkinLedMode.SOLID_GREEN);
    // }

    // private void setRedSolid() {
    //     led.setMode(BlinkinLedMode.SOLID_RED);
    // }

    // private void setRedBlinking() {
    //     led.setMode(BlinkinLedMode.TWO_STROBE);
    // }

    // @Override
    // public void periodic() {
    // }
}
