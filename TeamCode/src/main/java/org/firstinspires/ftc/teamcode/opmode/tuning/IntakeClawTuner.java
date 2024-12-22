package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Claw Tuner", group = "Tuning")
public class IntakeClawTuner extends LinearOpMode {
    public static double intakeL= 0;
    public static double intakeR = 0;
    public static double intakeMid = 0;
    public static double grab = 0;
    public static double release = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo iYaw = new SimpleServo(hardwareMap, "iYaw", 0, 180);
        SimpleServo iClaw = new SimpleServo(hardwareMap, "iClaw", 0, 180);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                iYaw.setPosition(intakeL);
            } else if (gamepad1.b) {
                iYaw.setPosition(intakeR);
            } else if (gamepad1.x) {
                iYaw.setPosition(intakeMid);
            } else if (gamepad1.right_bumper) {
                iClaw.setPosition(grab);
            } else if (gamepad1.left_bumper) {
                iClaw.setPosition(release);
            }
            telemetry.update();
        }
    }
}
