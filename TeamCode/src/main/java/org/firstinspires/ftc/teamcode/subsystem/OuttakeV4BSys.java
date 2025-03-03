package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
@Config
public class OuttakeV4BSys extends SubsystemBase {

    public static double PITCH_HOME = 0.0;
    public static double PITCH_AWAY = 0.95;
    public static double PITCH_MID = 0.5;
    public static double ARM_MID = 0.5;
    public static double ARM_HOME = 0.67;
    public static double ARM_AWAY= 0.3;

    public static double ARM_HOME_SPECIMEN = 0.18;
    public static double PITCH_HOME_SPECIMEN = 0.9;

    public static double ARM_TOUCH = 0.35;
    public static double PITCH_TOUCH = 0.6;

    SimpleServo pitch, arm;
    public OuttakeV4BSys(SimpleServo pitch, SimpleServo arm) {
        this.pitch = pitch;
        this.arm = arm;
        this.pitch.setPosition(0.4);
        this.arm.setPosition(0.6);
    }

    public Command setArm(double position) {
        return new InstantCommand(() -> arm.setPosition(position), this);
    }

    public Command setPitch(double position) {
        return new InstantCommand(() -> pitch.setPosition(position), this);
    }

    public void move(double pos,double rot) {
        arm.setPosition(pos);
        pitch.setPosition(rot);
    }

    public Command mid() {
        return new InstantCommand(()->{
            move(ARM_MID, PITCH_MID);
        }, this);
    }

    public Command specimen() {
        return new InstantCommand(()->{
            move(ARM_HOME_SPECIMEN, PITCH_HOME_SPECIMEN);
        }, this);
    }

    public Command home() {
        return new InstantCommand(() -> move(ARM_HOME, PITCH_HOME), this);
    }
    
    public Command away() {
        return new InstantCommand(() -> move(ARM_AWAY, PITCH_AWAY), this);
    }

    public Command specimenScore() {
        return new InstantCommand(() -> move(0.2, PITCH_AWAY), this);
    }

    public Command touch() {
        return new InstantCommand(() -> move(ARM_TOUCH, PITCH_TOUCH), this);
    }
}
