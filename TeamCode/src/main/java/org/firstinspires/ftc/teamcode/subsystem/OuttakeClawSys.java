package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
@Config
public class OuttakeClawSys extends SubsystemBase {

    public static double GRAB = 0.3;
    public static double SPECIMAN_GRAB = 0.3;
    public static double SUPER_GRAB=0.3;
    public static double RELEASE = 0.0;
    public static double SUPER_RELEASE = 0;
    private final SimpleServo claw;


    public OuttakeClawSys(SimpleServo claw) {
        this.claw = claw;
    }

    public Command grab() {
        return new InstantCommand(() -> claw.setPosition(GRAB), this);
    }

    public Command superGrab() {return new InstantCommand(()-> claw.setPosition(SUPER_GRAB), this);}

    public Command release() {
        return new InstantCommand(() -> claw.setPosition(RELEASE), this);
    }

    public Command superRelease() {
        return new InstantCommand(() -> claw.setPosition(SUPER_RELEASE), this);
    }

    public Command grabSpecimen() {
        return new InstantCommand(() -> claw.setPosition(SPECIMAN_GRAB), this);
    }
}

