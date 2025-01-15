package org.firstinspires.ftc.teamcode.util.pathfinder;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.control.Controller;
import xyz.devmello.voyager.control.GenericTurnController;
import xyz.devmello.voyager.follower.FollowerGenerator;
import xyz.devmello.voyager.follower.generators.GenericFollowerGenerator;
import xyz.devmello.voyager.geometry.PointXY;
import xyz.devmello.voyager.geometry.Rectangle;
import xyz.devmello.voyager.robot.components.Motor;
import xyz.devmello.voyager.drive.MecanumDrive;
import xyz.devmello.voyager.robot.Drive;
import xyz.devmello.voyager.robot.Odometry;
import xyz.devmello.voyager.robot.Robot;
import xyz.devmello.voyager.robot.components.AbstractMotor;
import xyz.devmello.voyager.zones.Zone;
@Config
public class PathfinderRobot {
    public static double coefficient = 0.05;
    private DcMotor dcMotorFrontRight;
    private DcMotor dcMotorFrontLeft;
    private DcMotor dcMotorBackRight;
    private DcMotor dcMotorBackLeft;

    private Motor motorFrontRight;
    private Motor motorFrontLeft;
    private Motor motorBackRight;
    private Motor motorBackLeft;

    private Drive drive;
    private PinpointOdometry odometry;
    private Robot robot;

    private static final Controller turnController = new GenericTurnController(coefficient);
    private static final FollowerGenerator followerGenerator = new GenericFollowerGenerator(turnController);

    private Voyager voyager;

    public void init(HardwareMap map) {

        //Zone zone = new Zone(new Rectangle(new PointXY(10,10), new PointXY(10,15), new PointXY(15,15), new PointXY(15,10)), () -> {}, () -> {}, () -> {});

        dcMotorFrontRight = map.get(DcMotor.class, "fr");
        dcMotorFrontLeft = map.get(DcMotor.class, "fl");
        dcMotorBackRight = map.get(DcMotor.class, "br");
        dcMotorBackLeft = map.get(DcMotor.class, "bl");

        dcMotorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        dcMotorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight = new AbstractMotor(
                dcMotorFrontRight::setPower,
                dcMotorFrontRight::getPower
        );
        motorFrontLeft = new AbstractMotor(
                dcMotorFrontLeft::setPower,
                dcMotorFrontLeft::getPower
        );
        motorBackRight = new AbstractMotor(
                dcMotorBackRight::setPower,
                dcMotorBackRight::getPower
        );
        motorBackLeft = new AbstractMotor(
                dcMotorBackLeft::setPower,
                dcMotorBackLeft::getPower
        );

        drive = new MecanumDrive(
                motorFrontRight,
                motorFrontLeft,
                motorBackRight,
                motorBackLeft
        );

        odometry = new PinpointOdometry(map);

        robot = new Robot(drive, odometry);

        voyager = new Voyager(robot, followerGenerator);

        //voyager.addZone("test", zone);
    }

    public Pose2D pos(){
        return odometry.getPos();
    }

    public void update() {
        odometry.update();
    }

    public Voyager pathfinder() {
        return this.voyager;
    }
}
