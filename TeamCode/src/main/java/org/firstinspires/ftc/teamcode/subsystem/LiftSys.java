package org.firstinspires.ftc.teamcode.subsystem;

import android.text.method.Touch;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.ProfiledPIDFController;

import java.util.function.DoubleSupplier;
@Config
public class LiftSys extends SubsystemBase {
    private final MotorEx top, bottem;
    private final Motor.Encoder encoder;
    private final TouchSensor touch;

    public static int NONE = 0;
    public static int LOW_RUNG = 0;
    public static int HIGH_RUNG = 740;
    public static int LOW_BUCKET = 740;
    public static int HIGH_BUCKET = 2000;

    public static double kP = 0.009;
    public static double kI = 0.0000;
    public static double kD = 0.00000;
    public static double kG = 0.032;
    public static double MAX_VEL = 10000;
    public static double MAX_ACCEL = 10000;

    //private final PIDController controller = new PIDController(kP,kI,kD);
    private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL));

    public static int threshold = 30;
    public static double slowFactor = 1.5;
    private final DoubleSupplier doubleSupplier;
    private int currentTarget = 0;

    public LiftSys(MotorEx top, MotorEx bottem, DoubleSupplier doubleSupplier, TouchSensor touch) {
        this.top = top;
        this.bottem = bottem;
        this.touch = touch;
        encoder = bottem.encoder;
        this.doubleSupplier = doubleSupplier;
        top.motorEx.setCurrentAlert(7, CurrentUnit.AMPS);
        bottem.motorEx.setCurrentAlert(7, CurrentUnit.AMPS);
    }

    public Command goTo(int target) {return setTarget(target).andThen(new WaitUntilCommand(this::atTarget));}

    public Command setTarget(int target) {return new InstantCommand(() -> {currentTarget = target;controller.setGoal(target);});}

    public boolean atTarget() {return Math.abs(encoder.getPosition() - currentTarget) < threshold;}

    public int getPosition() {return encoder.getPosition();}

    public int getCurrentTarget() {
        return currentTarget;
    }

    @Override
    public void periodic() {
        if (!(top.motorEx.isOverCurrent() && bottem.motorEx.isOverCurrent())) {
            if (touch.isPressed()) {
                encoder.reset();
            }
            if (doubleSupplier.getAsDouble() != 0) {
                top.set(doubleSupplier.getAsDouble()/slowFactor);
                bottem.set(doubleSupplier.getAsDouble()/slowFactor);

                controller.setGoal(encoder.getPosition());
            } else {
                double output = controller.calculate(encoder.getPosition()) + kG;
                top.set(output);
                bottem.set(output);
            }
        }
    }
}
