package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Intake V4B Tuner", group = "Tuning")
public class IntakeV4BTuner extends LinearOpMode {

    public static double intakePosHome = 0.5;
    public static double intakePosOut = 0;
    public static double intakePosMid = 0.3;

    public static double pitchPosHome = 0;
    public static double pitchPosOut = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo pos = new SimpleServo(hardwareMap, "iPos", 0, 255);
        SimpleServo pitch = new SimpleServo(hardwareMap, "iPitch", 0, 255);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                pos.setPosition(intakePosHome);
            } else if (gamepad1.b) {
                pos.setPosition(intakePosOut);
            } else if (gamepad1.x) {
                pos.setPosition(intakePosMid);
            } else if (gamepad1.left_bumper) {
                pitch.setPosition(pitchPosHome);
            } else if (gamepad1.right_bumper) {
                pitch.setPosition(pitchPosOut);
            }
            telemetry.update();
        }
    }
}
