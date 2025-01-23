package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import xyz.devmello.voyager.geometry.PointXYZ;

import java.util.function.DoubleSupplier;

@Config
public class DriveSys extends SubsystemBase {
    public final PinpointDrive drive;

    public static boolean AUTOMATION = false;
    public static double slow = 1;
    public static double slowT = 0.7;

    public DriveSys(HardwareMap hardwareMap) {
        drive = new PinpointDrive(hardwareMap, Robot.startPose);
        drive.setCoast();
    }

    public Command drive(DoubleSupplier f, DoubleSupplier s, DoubleSupplier t) {
        return new RunCommand(
                ()-> {
                    if (!AUTOMATION) {
                        drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        f.getAsDouble() * slow,
                                        -s.getAsDouble()* slow
                                ),
                                -t.getAsDouble() * slow * slowT
                        ));
                    }
                },this
       //         () -> drive.driveRobotCentric(-1 * s.getAsDouble() * slow, -1 * f.getAsDouble() * slow, -1 * t.getAsDouble() * slowT * slow),this
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
