package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "OuttakeV4BTuner", group = "tuning")
public class OuttakeV4BTuner extends LinearOpMode {
    public static double PITCH_HOME = 0.2;
    public static double PITCH_AWAY = 0.5;
    public static double ARM_HOME = 1;
    public static double ARM_AWAY= 0;
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo pitch = new SimpleServo(hardwareMap, "oPitch", 0, 255);
        SimpleServo arm = new SimpleServo(hardwareMap, "oPos", 0, 255);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                pitch.setPosition(PITCH_HOME);
            } else if (gamepad1.b) {
                pitch.setPosition(PITCH_AWAY);
            }
            if (gamepad1.x) {
                arm.setPosition(ARM_HOME);
            } else if (gamepad1.y) {
                arm.setPosition(ARM_AWAY);
            }
        }
    }
}
