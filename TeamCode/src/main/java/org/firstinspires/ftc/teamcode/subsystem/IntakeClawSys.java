package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import org.firstinspires.ftc.teamcode.opencv.SampleTrackPipeline;
import org.firstinspires.ftc.teamcode.util.math.Precision;

import java.util.function.DoubleSupplier;

@Config
public class IntakeClawSys extends SubsystemBase {

    public static double MID = 0.36;
    public static double PINCH = 0.76;
    public static double SUPER_PINCH = 0.69;

    public static double YAW_MID = 0.53;
    public static double YAW_LEFT = 0;
    public static double YAW_RIGHT = 1;

    public static boolean TRACK = false;

    private final SimpleServo claw;
    private final SimpleServo yaw;

    private SampleTrackPipeline pipeline;

    public static boolean AUTO = false;

    DoubleSupplier LTrigger, RTrigger;

    private double lastPos = 0;

    public IntakeClawSys(SimpleServo claw, SimpleServo yaw, DoubleSupplier LTrigger, DoubleSupplier RTrigger) {
        this.claw = claw;
        this.yaw = yaw;
        claw.setPosition(PINCH);
        yaw.setPosition(YAW_MID);
        this.LTrigger = LTrigger;
        this.RTrigger = RTrigger;
    }

    public Command pinch() {return new InstantCommand(() -> claw.setPosition(PINCH));}
    public Command superPinch() {return new InstantCommand(() -> claw.setPosition(SUPER_PINCH));}
    public Command release() {return new InstantCommand(() -> claw.setPosition(MID));}
    public Command rotateYaw(double yawVal) {return new InstantCommand(() -> yaw.setPosition(yawVal));}

    public Command dropoff() {
        return new InstantCommand(()-> {
            claw.setPosition(PINCH);

        }).andThen(new WaitCommand(75)).andThen(new InstantCommand(()-> {
            yaw.setPosition(YAW_MID);
        }));
    }
    public Command intake() {
        return new InstantCommand(() -> {
            claw.setPosition(PINCH);
            yaw.setPosition(YAW_MID);
        });
    }

    public Command enableTracking() {
        return new InstantCommand(()-> TRACK = true);
    }

    public Command disableTracking() {
        return new InstantCommand(()-> TRACK = false);
    }

    public void setPipeline(SampleTrackPipeline pipeline) {
        this.pipeline = pipeline;
        if(this.pipeline == null) Log.println(Log.WARN, "pipeline null", "pipeline null");
    }

    public void updatePipeline() {
        if (pipeline.getAngle() == -1) return;
//        yaw.setPosition(Precision.calculateWeightedValue(YAW_LEFT, YAW_RIGHT, (pipeline.getAngle() % 179) / 180));

        yaw.setPosition(Math.round(Precision.calculateWeightedValue(YAW_LEFT, YAW_RIGHT, (pipeline.getAngle() % 179) / 180) * 5) / 5.0);
        lastPos = yaw.getAngle();
    }

    @Override
    public void periodic() {
        if (RTrigger.getAsDouble() > 0) {
            yaw.rotateBy(0.01);
        } else if (LTrigger.getAsDouble() > 0) {
            yaw.rotateBy(-0.01);
        } else {
            if (TRACK && pipeline != null && !AUTO) {
                updatePipeline();
            }
        }
    }


}
