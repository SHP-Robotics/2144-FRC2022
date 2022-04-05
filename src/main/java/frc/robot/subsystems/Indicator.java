package frc.robot.subsystems;

import static frc.robot.RobotContainer.*;
import static frc.robot.Constants.Indicator.*;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indicator extends SubsystemBase {
    // private final BlinkinLedDriver led = new BlinkinLedDriver(0);

    public void update(int numBallsIndexed, boolean targetLocked, boolean driverOverride) {
        if ((numBallsIndexed > 0 && targetLocked) || (numBallsIndexed > 0 && driverOverride)) {
            startRumbleLeft();
            if (numBallsIndexed == 2) {
                startRumbleRight();
            } else
                stopRumbleRight();
        } else
            stopRumble();
    }

    private void startRumbleLeft() {
        controller.setRumble(RumbleType.kLeftRumble, kRumbleValue);
    }

    private void startRumbleRight() {
        controller.setRumble(RumbleType.kRightRumble, kRumbleValue);
    }

    private void stopRumbleLeft() {
        controller.setRumble(RumbleType.kLeftRumble, 0);
    }

    private void stopRumbleRight() {
        controller.setRumble(RumbleType.kRightRumble, 0);
    }

    private void stopRumble() {
        stopRumbleLeft();
        stopRumbleRight();
    }
}
