package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeV4bSys.*;
import static org.firstinspires.ftc.teamcode.subsystem.OuttakeV4BSys.*;

public class MainOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();
        register(driveSys, extendoSys, liftSys, intakeClawSys, outtakeClawSys, timeSys, intakeV4bSys, outtakeV4bSys, transmissionSys);
        intakeClawSys.setPipeline(pipeline);
        try {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    telemetry.addData("Camera has been initialized", true);
                    telemetry.update();
                }

                @Override
                public void onError(int errorCode) {
                    Log.e("OpenCv", "Error opening camera: " + errorCode);
                }
            });
        } catch (Exception e) {
            Log.e("OpenCv", "Error opening camera");
            Log.e("OpenCv", e.getMessage());
        }

        gb1(GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(outtakeClawSys.grab(), outtakeClawSys.release());

        gb1(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(
                new ParallelCommandGroup(
                        extendoSys.goTo(ExtendoSys.EXTENDO_MAX),
                        intakeV4bSys.intake(),
                        outtakeV4bSys.mid(),
                        new InstantCommand(pipeline::enableTracking),
                        intakeClawSys.intake()
                ),
                new SequentialCommandGroup(
                        intakeV4bSys.goToRoll(ROLL_OUT),
                        intakeClawSys.release(),
                        new WaitCommand(150),
                        new InstantCommand(pipeline::disableTracking),
                        intakeV4bSys.goToPos(POS_DOWN - 0.05),
                        new WaitCommand(100),
                        intakeClawSys.pinch(),
                        new WaitCommand(100),
                        intakeClawSys.dropoff(),
                        new WaitCommand(150),
                        intakeV4bSys.dropOff(),
                        extendoSys.goTo(ExtendoSys.EXTENDO_HOME)
                )
        );

        gb1(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(
                new SequentialCommandGroup(
                        outtakeClawSys.release(),
                        new WaitCommand(300),
                        outtakeV4bSys.mid(),
                        liftSys.goTo(LiftSys.NONE)
                ),
                new SequentialCommandGroup(
                        outtakeV4bSys.setPitch(PITCH_HOME),
                        new WaitCommand(200),
                        outtakeV4bSys.setArm(ARM_HOME),
                        new WaitCommand(200),
                        outtakeClawSys.grab(),
                        new WaitCommand(150),
                        intakeClawSys.release(),
                        new WaitCommand(50),
                        outtakeV4bSys.away()
                )
        );

        gb1(GamepadKeys.Button.A).whenPressed(liftSys.goTo(LiftSys.NONE));
        gb1(GamepadKeys.Button.Y).whenPressed(
                liftSys.goTo(LiftSys.HIGH_BUCKET)
        );

        gb1(GamepadKeys.Button.X).whenPressed(liftSys.goTo(LiftSys.HIGH_RUNG));
        gb1(GamepadKeys.Button.B).toggleWhenPressed(
                        outtakeV4bSys.away(),
                        outtakeV4bSys.specimen()
        );

        gb1(GamepadKeys.Button.DPAD_UP).toggleWhenPressed(
                new ParallelCommandGroup(
                        extendoSys.goTo(ExtendoSys.EXTENDO_POP),
                        intakeV4bSys.intake(),
                        new InstantCommand(pipeline::enableTracking),
                        intakeClawSys.intake()
                ),
                new SequentialCommandGroup(
                        intakeClawSys.release(),
                        new WaitCommand(150),
                        new InstantCommand(pipeline::disableTracking),
                        intakeV4bSys.goToPos(POS_DOWN - 0.05),
                        new WaitCommand(100),
                        intakeClawSys.pinch(),
                        new WaitCommand(100),
                        intakeClawSys.dropoff(),
                        new WaitCommand(150),
                        intakeV4bSys.dropOff(),
                        new WaitCommand(150),
                        extendoSys.goTo(ExtendoSys.EXTENDO_HOME)
                )
        );

        //TODO: For Automations, add the entire command and figure out how to dynamically generate commands.
        //  Goal is to delete the Robot Class.
        //  Re-enable the automations as well before competition.

//        gb1(GamepadKeys.Button.START).whenPressed(
//                new InstantCommand(()-> driveSys.drive.pose = new Pose2d(new Vector2d(63.3, -62.2), Math.toRadians(90)))
//        );

//        gb1(GamepadKeys.Button.DPAD_DOWN).toggleWhenPressed(
//                new InstantCommand(()-> Robot.specimenPickup(driveSys.drive, outtakeV4bSys, outtakeClawSys, liftSys)),
//                new InstantCommand(()-> Robot.specimenScore(driveSys.drive, outtakeV4bSys, outtakeClawSys, liftSys))
//        );
//
//        gb1(GamepadKeys.Button.BACK).whenPressed(()->DriveSys.AUTOMATION = false);
//
//        gb1(GamepadKeys.Button.DPAD_RIGHT).toggleWhenPressed(
//                new InstantCommand(()-> Robot.samplePickup(driveSys.drive, outtakeV4bSys, outtakeClawSys, liftSys, extendoSys, intakeV4bSys, intakeClawSys, pipeline))
//        );


        //TODO: Optimize the button placements to make everything a lot easier.
        gb1(GamepadKeys.Button.START).whenPressed(
                new SequentialCommandGroup(
                        liftSys.goTo(LiftSys.HANG),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                liftSys.vibrate(1500, 0.2),
                                transmissionSys.shiftUp()
                        )
                )
        );

        driveSys.setDefaultCommand(driveSys.drive(
                gamepadEx1::getLeftY,
                gamepadEx1::getLeftX,
                gamepadEx1::getRightX
        ));

        transmissionSys.setDefaultCommand(transmissionSys.manualControl(gamepadEx1::getRightY));
    }
}
