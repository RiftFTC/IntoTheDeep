package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
@Config
public class IntakeV4bSys extends SubsystemBase {
    private final SimpleServo pr, pitch;

    public static double POS_MID = 0.51;
    public static double POS_DOWN = 0.44;
    public static double POS_IN = 0.83;
    public static double POS_SPECIMEN_OUT = 0.65;
    public static double POS_SPECIMEN_IN = 0.66;

    public static double ROLL_IN = 0.5;
    public static double ROLL_MID = 0.8; //0.76
    public static double ROLL_OUT = 0.8;


    public static double ROLL_OUT_SPECIMEN = 0.56;

    public static double ROLL_IN_SPECIMEN = 0.3;

    public enum State{
        MID,
        DOWN,
        IN,
        OUT
    }
    public static State state = State.IN;

    public IntakeV4bSys(SimpleServo pr, SimpleServo pitch) {
        this.pr = pr;
        this.pitch = pitch;
        pr.setPosition(POS_IN);
        pitch.setPosition(ROLL_IN);
    }

    public Command goToPos(double pos) {return new InstantCommand(() -> pr.setPosition(pos));}
    public Command goToRoll(double pos) {return new InstantCommand(() -> pitch.setPosition(pos));}

    public Command dropOff() {return new ParallelCommandGroup(goToPos(POS_IN), goToRoll(ROLL_IN));}
    public Command intake() {return new ParallelCommandGroup(goToRoll(ROLL_MID),goToPos(POS_MID));}

    public Command specimenIntake() {return new ParallelCommandGroup(goToRoll(ROLL_OUT_SPECIMEN),goToPos(POS_SPECIMEN_OUT));}

}
