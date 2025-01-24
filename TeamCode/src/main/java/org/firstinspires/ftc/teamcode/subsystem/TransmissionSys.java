package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.function.DoubleSupplier;
@Config
public class TransmissionSys extends SubsystemBase {
    private final SimpleServo transmissionServo;
    private final MotorEx hang;
    public static double TRANSMISSION_UP = 0.3;
    public static double TRANSMISSION_DOWN = 0;
    private final Motor.Encoder encoder;
    private boolean hanging = false;

    public TransmissionSys(SimpleServo transmissionServo, MotorEx hangMotor, Motor.Encoder encoder) {
        this.transmissionServo = transmissionServo;
        this.hang = hangMotor;
        this.encoder = encoder;
        transmissionServo.setPosition(TRANSMISSION_DOWN);
        hangMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public Command shiftUp() {
        return new InstantCommand(() -> {transmissionServo.setPosition(TRANSMISSION_UP);hanging=true;});
    }

    public Command manualControl(DoubleSupplier manualControl) {
        return new RunCommand(() -> {
            int position = encoder.getPosition();
            if (hanging && !(position < 250) && !(position > 1750)) {
                hang.set(manualControl.getAsDouble());
            } else if (position < 250 && manualControl.getAsDouble() > 0 && hanging) {
                hang.set(manualControl.getAsDouble());
            } else if (position > 1750 && manualControl.getAsDouble() < 0 && hanging) {
                hang.set(manualControl.getAsDouble());
            } else {
                hang.set(0);
            }
        }, this);
    }
}
