package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config
@TeleOp(name = "Outtake Claw Tuner", group = "Tuning")
public class OuttakeClawTuner extends LinearOpMode {

    public static double GRAB = 0.5;
    public static double RELEASE = 0.45;
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo claw = new SimpleServo(hardwareMap, "oClaw", 0, 180);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                claw.setPosition(GRAB);
            } else if (gamepad1.b) {
                claw.setPosition(RELEASE);
            }
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.update();
        }
    }
}
