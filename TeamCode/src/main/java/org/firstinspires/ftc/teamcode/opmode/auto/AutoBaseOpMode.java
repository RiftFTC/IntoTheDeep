package org.firstinspires.ftc.teamcode.opmode.auto;

import android.app.ActivityManager;
import android.content.Context;
import android.util.Log;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opencv.SampleTrackPipeline;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import xyz.devmello.voyager.time.ElapsedTimer;

import java.util.List;
import java.util.Objects;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

public class AutoBaseOpMode extends OpMode {
    protected GamepadEx gamepadEx1, gamepadEx2;
    protected TouchSensor touch;
    protected MotorEx fl, fr, bl, br, lil, lir, hang;
    protected ElapsedTimer elapsedTimer;
    protected SimpleServo ipr, iPitch, extL, extR, iClaw, iYaw, oClaw, oPitch, oPos, transmission;
    protected PinpointDrive drive;
    protected ExtendoSys extendoSys;
    protected LiftSys liftSys;
    protected IntakeV4bSys intakeV4bSys;
    protected IntakeClawSys intakeClawSys;
    protected OuttakeClawSys outtakeClawSys;
    protected TimeSys timeSys;
    protected OpenCvCamera camera;
    protected OuttakeV4BSys outtakeV4bSys;
    protected SampleTrackPipeline pipeline;
    public enum TEAM {
        RED,
        BLUE
    }
    public BaseOpMode.TEAM team;
    public void setTeam() {
        team = BaseOpMode.TEAM.BLUE;
    }
    private static final ActivityManager activityManager = (ActivityManager) AppUtil.getDefContext().getSystemService(Context.ACTIVITY_SERVICE);
    private ActivityManager.MemoryInfo memoryInfo = new ActivityManager.MemoryInfo();
    List<LynxModule> allHubs;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        setTeam();
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        initHw();
        configHw();
        initSys();
        setupMisc();
        register(extendoSys, liftSys, intakeClawSys, outtakeClawSys, timeSys, intakeV4bSys, outtakeV4bSys);
        intakeClawSys.setPipeline(pipeline);
        IntakeClawSys.AUTO = true;
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
            Log.e("OpenCv", Objects.requireNonNull(e.getMessage()));
        }
    }

    public void initHw() {
        fl = new MotorEx(hardwareMap, "fl");
        fr = new MotorEx(hardwareMap, "fr");
        bl = new MotorEx(hardwareMap, "bl");
        br = new MotorEx(hardwareMap, "br");
        lil = new MotorEx(hardwareMap, "lil");
        lir = new MotorEx(hardwareMap, "lir");
        hang = new MotorEx(hardwareMap, "hang");
        extL = new SimpleServo(hardwareMap, "extL", 0, 270);
        extR = new SimpleServo(hardwareMap, "extR", 0, 270);
        ipr = new SimpleServo(hardwareMap, "iPos", 0, 255);
        iPitch = new SimpleServo(hardwareMap, "iPitch", 0, 255);
        iClaw = new SimpleServo(hardwareMap, "iClaw", 0, 180);
        iYaw = new SimpleServo(hardwareMap, "iYaw", 0, 180);
        oClaw = new SimpleServo(hardwareMap, "oClaw", 0, 180);
        oPitch = new SimpleServo(hardwareMap, "oPitch", 0, 180);
        oPos = new SimpleServo(hardwareMap, "oPos", 0, 180);
        transmission = new SimpleServo(hardwareMap, "trans", 0, 180);
        touch = hardwareMap.get(TouchSensor.class, "touch");
    }

    public void configHw() {
        extR.setInverted(true);
    }

    public void initSys() {
        extendoSys = new ExtendoSys(extL,extR);
        liftSys = new LiftSys(lil, lir, gamepadEx1::getRightY, touch);
        intakeV4bSys = new IntakeV4bSys(ipr, iPitch);
        intakeClawSys = new IntakeClawSys(iClaw, iYaw, ()-> gamepadEx1.getTrigger(LEFT_TRIGGER), () -> gamepadEx1.getTrigger(RIGHT_TRIGGER));
        outtakeClawSys = new OuttakeClawSys(oClaw);
        outtakeV4bSys = new OuttakeV4BSys(oPitch, oPos);
        timeSys = new TimeSys();
    }

    public void setupMisc() {
        elapsedTimer = new ElapsedTimer();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        pipeline = new SampleTrackPipeline(team);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        camera.setPipeline(pipeline);
        allHubs = hardwareMap.getAll(LynxModule.class);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        super.stop();
        elapsedTimer.toString() ;
        IntakeClawSys.AUTO = false;
        Robot.startPose = drive.pose;
    }

    @Override
    public void start() {
        elapsedTimer.start();
    }
}
