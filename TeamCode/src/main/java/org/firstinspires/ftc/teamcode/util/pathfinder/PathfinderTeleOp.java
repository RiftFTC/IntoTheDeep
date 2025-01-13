package org.firstinspires.ftc.teamcode.util.pathfinder;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.geometry.Translation;

@TeleOp(name = "PathfinderTeleOp", group = "default")
public class PathfinderTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // initialize our pathfinder instance, so we can do some
        // pretty cool pathfinding
        PathfinderRobot robot = new PathfinderRobot();
        robot.init(hardwareMap);
        Voyager voyager = robot.pathfinder();

        // wait for the game to start
        waitForStart();

        // while the TeleOp mode is active (this should be just about
        // the entire time)
        while (opModeIsActive()) {
            if (voyager.isActive()) {
                // pathfinder is already active. if the user presses the
                // start button, we should cancel pathfinder - this is
                // like a manual override.
                if (gamepad1.start) {
                    voyager.clear();
                }
            } else {
                // check to see if the ABXY buttons are pressed.
                // if any of them are, go to a point specific to that button.
                PointXYZ target = null;

                // these points can be whatever you'd like them to be
                if (gamepad1.a) {
                    target = new PointXYZ(10, 10, Angle.fromDeg(45));
                } else if (gamepad1.b) {
                    target = new PointXYZ(20, 20, Angle.fromDeg(45));
                } else if (gamepad1.x) {
                    target = new PointXYZ(30, 30, Angle.fromDeg(45));
                } else if (gamepad1.y) {
                    target = new PointXYZ(40, 40, Angle.fromDeg(45));
                }

                // if the target is NOT null, tell pathfinder to go to
                // that target.
                if (target != null) {
                    // we'll need to tick pathfinder for this to actually
                    // have any effect on anything
                    voyager.goTo(target);
                }
            }

            // tick/update pathfinder - this must be run every update cycle!
            voyager.tick();

            // check again to see if pathfinder is NOT active
            // this state can change, so we need another conditional
            if (!voyager.isActive()) {
                // pathfinder isn't doing anything, we should drive the robot
                // using joystick inputs. create a translation and set the
                // robot to move according to that translation
                Translation translation = new Translation(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                );

                // set the translation to the chassis
                voyager.getDrive().setTranslation(translation);
            }
        }
    }
}