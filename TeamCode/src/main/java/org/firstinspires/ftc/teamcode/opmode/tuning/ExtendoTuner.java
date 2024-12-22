package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.ExtendoSys;

@Config
@TeleOp(name = "Extendo Tuner", group = "Tuning")
public class ExtendoTuner extends LinearOpMode {

    public static double extendoLHome = 0.05;
    public static double extendoRHome = 0;

    public static double extendoLMax = 1;
    public static double extendoRMax = 1;

    public static double extendoLMiddle = 0.5;
    public static double extendoRMiddle = 0.5;

    public static double extendoLPop = 0;
    public static double extendoRPop = 0;

    @Override
    public void runOpMode() {
        SimpleServo extendoL = new SimpleServo(hardwareMap, "extL", 0, 270);
        SimpleServo extendoR = new SimpleServo(hardwareMap, "extR", 0, 270);
        extendoR.setInverted(true);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
//            if (gamepad1.a) {
//                extendoL.setPosition(extendoLHome);
//            } else if (gamepad1.b) {
//                extendoL.setPosition(extendoLMiddle);
//            } else if (gamepad1.x) {
//                extendoL.setPosition(extendoLMax);
//            } else if (gamepad1.y) {
//                extendoL.setPosition(extendoLPop);
//            } else if (gamepad1.dpad_up) {
//                extendoR.setPosition(extendoRMax);
//            } else if (gamepad1.dpad_down) {
//                extendoR.setPosition(extendoRHome);
//            } else if (gamepad1.dpad_left) {
//                extendoR.setPosition(extendoRPop);
//            } else if (gamepad1.dpad_right) {
//                extendoR.setPosition(extendoRMiddle);
//            }

            if(gamepad2.right_bumper) {
                extendoL.setPosition(ExtendoSys.EXTENDO_HOME);
                extendoR.setPosition(ExtendoSys.EXTENDO_HOME);
            }
            if (gamepad2.left_bumper) {
                extendoL.setPosition(ExtendoSys.EXTENDO_MAX);
                extendoR.setPosition(ExtendoSys.EXTENDO_MAX);
            }
            telemetry.update();
        }
    }
}
