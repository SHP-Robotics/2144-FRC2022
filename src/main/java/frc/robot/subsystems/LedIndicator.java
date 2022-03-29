package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.BlinkinLedDriver;
import frc.robot.utils.BlinkinLedDriver.BlinkinLedMode;

public class LedIndicator extends SubsystemBase {
    private final BlinkinLedDriver led = new BlinkinLedDriver(0);

    public void update(boolean ballIndexed, boolean targetLocked, boolean driverOverride) {
        if (driverOverride) {
            this.setBlue();
            return;
        }

        if (ballIndexed && targetLocked) {
            setGreenBlinking();
        } else if (ballIndexed && !targetLocked) {
            setGreenSolid();
        } else if (!ballIndexed && targetLocked) {
            setRedBlinking();
        } else setRedSolid();
    }

    private void setBlue() {
        led.setMode(BlinkinLedMode.SOLID_BLUE);
    }

    private void setGreenBlinking() {
        led.setMode(BlinkinLedMode.ONE_STROBE);
    }

    private void setGreenSolid() {
        led.setMode(BlinkinLedMode.SOLID_GREEN);
    }

    private void setRedSolid() {
        led.setMode(BlinkinLedMode.SOLID_RED);
    }

    private void setRedBlinking() {
        led.setMode(BlinkinLedMode.TWO_STROBE);
    }

    // @Override
    // public void periodic() {
    // }
}
