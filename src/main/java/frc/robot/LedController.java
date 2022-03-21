package frc.robot;

import frc.robot.utils.BlinkinLedDriver;
import frc.robot.utils.BlinkinLedDriver.BlinkinLedMode;

public final class LedController {
    public static final BlinkinLedDriver led = new BlinkinLedDriver(0);

    public static void setGreenBlinking() {
        led.setMode(BlinkinLedMode.ONE_HEARTBEAT_FAST);
    }

    public static void setGreenSolid() {
        led.setMode(BlinkinLedMode.SOLID_GREEN);
    }

    public static void setRedSolid() {
        led.setMode(BlinkinLedMode.SOLID_RED);
    }

    public static void setRedBlinking() {
        led.setMode(BlinkinLedMode.TWO_HEARTBEAT_FAST);
    }
}
