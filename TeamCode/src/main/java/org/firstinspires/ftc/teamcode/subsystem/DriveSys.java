package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.function.DoubleSupplier;

@Config
public class DriveSys extends SubsystemBase {
    public final MecanumDrive drive;

    public static double slow = 1;
    public static double slowT = 0.7;

    public DriveSys(MotorEx fl, MotorEx fr, MotorEx bl, MotorEx br) {
        drive = new MecanumDrive(fl, fr, bl, br);
    }

    public Command drive(DoubleSupplier s, DoubleSupplier f, DoubleSupplier t) {
        return new RunCommand(
                () -> drive.driveRobotCentric(-1 * s.getAsDouble() * slow, -1 * f.getAsDouble() * slow, -1 * t.getAsDouble() * slowT * slow),this
        );
    }
//
//    public Command drive(DoubleSupplier s, DoubleSupplier f, DoubleSupplier t) {
//        return drive(s,f,t,1);
//    }

    public Command slow(double slow) {
        return new InstantCommand(() -> {
            DriveSys.slow = slow;
        });
    }
}
