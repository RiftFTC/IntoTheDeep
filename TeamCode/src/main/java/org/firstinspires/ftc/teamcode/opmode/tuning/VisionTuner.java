package org.firstinspires.ftc.teamcode.opmode.tuning;

import android.app.ActivityManager;
import android.content.Context;
import android.util.Log;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.SampleTrackPipeline;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.IntakeClawSys;
import org.firstinspires.ftc.teamcode.subsystem.IntakeV4bSys;
import org.firstinspires.ftc.teamcode.util.math.Precision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeClawSys.YAW_LEFT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeClawSys.YAW_RIGHT;

@Config
@TeleOp(name = "Vision Tuner", group = "Tuning")
public class VisionTuner extends LinearOpMode {
    public static double yawL = 0;
    public static double yawR = 1;
    public static double yawM = 0.5;
    protected OpenCvCamera camera;
    protected SampleTrackPipeline pipeline;
    @Override
    public void runOpMode() throws InterruptedException {
        IntakeClawSys.TRACK = true;
        ActivityManager activityManager = (ActivityManager) hardwareMap.appContext.getSystemService(Context.ACTIVITY_SERVICE);
        ActivityManager.MemoryInfo memoryInfo = new ActivityManager.MemoryInfo();
        SimpleServo ipr = new SimpleServo(hardwareMap, "iPos", 0, 255);
        SimpleServo iPitch = new SimpleServo(hardwareMap, "iPitch", 0, 255);
        SimpleServo yaw = new SimpleServo(hardwareMap, "iYaw", 0, 180);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        pipeline = new SampleTrackPipeline(BaseOpMode.TEAM.BLUE);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        camera.setPipeline(pipeline);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
        } catch (Exception e) {
            Log.e("OpenCv", "Error opening camera");
            Log.e("OpenCv", e.getMessage());
        }
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            ipr.setPosition(IntakeV4bSys.POS_MID);
            iPitch.setPosition(0.8);
            if (gamepad1.a) {
                yaw.setPosition(yawL);
            } else if (gamepad1.b) {
                yaw.setPosition(yawR);
            } else if (gamepad1.x) {
                yaw.setPosition(yawM);
            }
//            } else {
//                yaw.setPosition(Precision.calculateWeightedValue(yawL, yawR, ((pipeline.getAngle() % 179) / 180)));
//            }
            yaw.setPosition(Math.round(Precision.calculateWeightedValue(YAW_LEFT, YAW_RIGHT, (pipeline.getAngle() % 179) / 180) * 5) / 5.0);
            activityManager.getMemoryInfo(memoryInfo);
            telemetry.addData("Memory Free", (float) memoryInfo.availMem / (float) memoryInfo.totalMem * 100.0F);
            telemetry.addData("Angle", pipeline.getAngle());
            telemetry.update();
        }
    }
}
