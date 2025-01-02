package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.GamepadServer;

@TeleOp
public class GamepadTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        GamepadServer gamepadServer = new GamepadServer(gamepad1);


        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
            telemetry.addData("Dpad Up", gamepad1.dpad_up);
            telemetry.addData("Dpad Down", gamepad1.dpad_down);
            telemetry.addData("Dpad Left", gamepad1.dpad_left);
            telemetry.addData("Dpad Right", gamepad1.dpad_right);
            telemetry.addData("A", gamepad1.a);
            telemetry.addData("B", gamepad1.b);
            telemetry.addData("X", gamepad1.x);
            telemetry.addData("Y", gamepad1.y);
            telemetry.addData("Start", gamepad1.start);
            telemetry.addData("Back", gamepad1.back);
            telemetry.addData("Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Right Bumper", gamepad1.right_bumper);
            telemetry.addData("Left Stick Button", gamepad1.left_stick_button);
            telemetry.addData("Right Stick Button", gamepad1.right_stick_button);
            telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", gamepad1.right_trigger);

            telemetry.update();
        }

        gamepadServer.shutdown();
    }

}
