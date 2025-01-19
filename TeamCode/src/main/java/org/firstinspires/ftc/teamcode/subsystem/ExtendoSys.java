package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;

@Config
public class ExtendoSys extends SubsystemBase {
    private final SimpleServo extendoL, extendoR;
    public static double EXTENDO_HOME = 0.15;
    public static double EXTENDO_MAX = 0.43;
    public static double EXTENDO_MIDDLE = 0.23;
    public static double EXTENDO_POP = 0.2;

    private double targetPos = 0.0;

    public enum State {
        EXTENDED,
        RETRACTED
    }
    public static State state = State.RETRACTED;

    public ExtendoSys(SimpleServo extendoL, SimpleServo extendoR) {
        this.extendoL = extendoL;
        this.extendoR = extendoR;
        this.extendoL.setPosition(EXTENDO_HOME);
        this.extendoR.setPosition(EXTENDO_HOME);
    }

    public Command goTo(double targetPos) {
        return new InstantCommand(() -> {
            this.targetPos = targetPos;
            if (targetPos == EXTENDO_HOME) {state = State.RETRACTED; DriveSys.slow = 1; DriveSys.slowT = 0.7;}
            else {state = State.EXTENDED;DriveSys.slow = 0.5; DriveSys.slowT = 0.5;}
            extendoL.setPosition(targetPos);
            extendoR.setPosition(targetPos);
        });
    }
}