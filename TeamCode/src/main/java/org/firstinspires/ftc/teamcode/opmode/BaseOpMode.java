package org.firstinspires.ftc.teamcode.opmode;

import android.annotation.SuppressLint;
import android.app.ActivityManager;
import android.content.Context;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opencv.SampleTrackPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.GamepadServer;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.util.Arrays;
import java.util.List;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

public class BaseOpMode extends CommandOpMode {
    protected GamepadEx gamepadEx1, gamepadEx2;
    protected MotorEx fl, fr, bl, br, lil, lir, hang;
    private double loopTime = 0;
    protected SimpleServo ipr, iPitch, extL, extR, iClaw, iYaw, oClaw, oPitch, oPos, transmission;
    protected TouchSensor touch;
    protected PinpointDrive pinpointDrive;
    protected DriveSys driveSys;
    protected ExtendoSys extendoSys;
    protected LiftSys liftSys;
    protected IntakeV4bSys intakeV4bSys;
    protected IntakeClawSys intakeClawSys;
    protected OuttakeClawSys outtakeClawSys;
    protected TimeSys timeSys;
    protected OpenCvCamera camera;
    protected OuttakeV4BSys outtakeV4bSys;
    protected SampleTrackPipeline pipeline;
    protected TransmissionSys transmissionSys;
    public enum TEAM {
        RED,
        BLUE
    }
    public TEAM team;
    public void setTeam() {
        team = TEAM.BLUE;
    }
    List<LynxModule> allHubs;
    GamepadServer gamepadServer;

    @Override
    public void initialize() {
        gamepadServer = new GamepadServer(gamepad1);
        setTeam();
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        initHw();
        configHw();
        initSys();
        setupMisc();
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
        pinpointDrive = new PinpointDrive(hardwareMap, Robot.startPose);
        driveSys = new DriveSys(fl, fr, bl ,br);
        extendoSys = new ExtendoSys(extL,extR);
        liftSys = new LiftSys(lil, lir, gamepadEx2::getRightY, touch);
        intakeV4bSys = new IntakeV4bSys(ipr, iPitch);
        intakeClawSys = new IntakeClawSys(iClaw, iYaw, ()-> gamepadEx1.getTrigger(LEFT_TRIGGER), () -> gamepadEx1.getTrigger(RIGHT_TRIGGER));
        outtakeClawSys = new OuttakeClawSys(oClaw);
        outtakeV4bSys = new OuttakeV4BSys(oPitch, oPos);
        timeSys = new TimeSys();
        transmissionSys = new TransmissionSys(transmission, hang, lir.encoder);
    }

    @SuppressLint("SdCardPath")
    public void setupMisc() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        pipeline = new SampleTrackPipeline(team);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        camera.setPipeline(pipeline);
        //FtcDashboard.getInstance().startCameraStream(camera, 0);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    protected GamepadButton gb1(GamepadKeys.Button button) {
        return gamepadEx1.getGamepadButton(button);
    }

    protected GamepadButton gb2(GamepadKeys.Button button) {
        return gamepadEx2.getGamepadButton(button);
    }

    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    @Override
    public void run() {
        super.run();
        pinpointDrive.updatePoseEstimate();
//        activityManager.getMemoryInfo(memoryInfo);
//        tad("Available Memory", (float) memoryInfo.availMem / (float) memoryInfo.totalMem * 100.0F);
        tad("ANGLE", pipeline.getAngle());
        tad("FPS", camera.getFps());
        tad("Hang Current", hang.motorEx.getCurrent(CurrentUnit.AMPS));
        tad("HANG POWER", hang.get());
        telemetry.addData("Encoder POS", lir.encoder.getPosition());
        telemetry.addData("lir", lir.motorEx.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("lil", lil.motorEx.getCurrent(CurrentUnit.AMPS));
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }

    @Override
    public void reset() {
        super.reset();
        gamepadServer.shutdown();
    }

}
