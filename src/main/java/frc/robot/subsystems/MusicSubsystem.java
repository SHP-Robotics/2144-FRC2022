package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MusicSubsystem extends SubsystemBase {
    private Orchestra orchestra;

    public MusicSubsystem(TalonFX motor) {
        orchestra = new Orchestra();
        orchestra.addInstrument(motor);
        orchestra.loadMusic("song.chrp");

    }

    public void play() {
        orchestra.play();
    }
}
