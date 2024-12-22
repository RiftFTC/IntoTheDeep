package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.function.DoubleSupplier;

public class TransmissionSys extends SubsystemBase {
    private final SimpleServo transmissionServo;
    private final MotorEx hang;
    public static double TRANSMISSION_UP = 0.5;
    public static double TRANSMISSION_DOWN = 0;
    private final TimeSys timeSys;

    public TransmissionSys(SimpleServo transmissionServo, MotorEx hangMotor, TimeSys timeSys) {
        this.transmissionServo = transmissionServo;
        this.hang = hangMotor;
        this.timeSys = timeSys;
        transmissionServo.setPosition(TRANSMISSION_DOWN);
        hangMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public Command transmissionUp() {
        return new InstantCommand(() -> transmissionServo.setPosition(TRANSMISSION_UP));
    }

    public Command manualControl(DoubleSupplier manualControl) {
        return new RunCommand(() -> {
            if (timeSys.getGameState() == TimeSys.GameState.HANG_PERIOD) {
                hang.set(manualControl.getAsDouble());
            }
        });
    }
}
