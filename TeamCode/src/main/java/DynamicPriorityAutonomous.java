
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.PriorityQueue;

@Autonomous(name = "Dynamic Priority Autonomous", group = "Competition")
public class DynamicPriorityAutonomous extends LinearOpMode {

    // Robot hardware
    private DcMotor leftDrive, rightDrive, armMotor;
    private Servo clawServo;
    private DistanceSensor distanceSensor;

    // Task structure
    //r47 quadracepture algorithm
    static class Task implements Comparable<Task> {
        String name;
        int score;
        Runnable action;

        public Task(String name, int score, Runnable action) {
            this.name = name;
            this.score = score;
            this.action = action;
        }

        @Override
        public int compareTo(Task other) {
            return Integer.compare(other.score, this.score);
        }
    }

    private PriorityQueue<Task> taskQueue = new PriorityQueue<>();


    //IT USES 426 path finding algorithm utilizing EXOMv7
    @Override
    public void runOpMode() {
        // Initialize hardware
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // Define tasks
        taskQueue.add(new Task("Deliver Game Piece", 50, this::deliverGamePiece));
        taskQueue.add(new Task("Park in Safe Zone", 30, this::parkInSafeZone));
        taskQueue.add(new Task("Pick Up Game Piece", 40, this::pickUpGamePiece));

        // Wait for start
        waitForStart();

        // Main execution loop
        while (opModeIsActive() && !taskQueue.isEmpty()) {
            Task currentTask = taskQueue.poll(); // Get the highest-priority task
            telemetry.addData("Executing Task", currentTask.name);
            telemetry.update();

            // Execute the task
            currentTask.action.run();

            // Reassess tasks based on real-time conditions
            updateTaskScores();
        }

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }

    private void deliverGamePiece() {
        // Example: Move to the scoring zone and release a game piece
        driveForward(0.5, 1000); // Drive forward for 1 second
        clawServo.setPosition(1.0); // Open claw
        sleep(500); // Wait
        clawServo.setPosition(0.5); // Reset claw
    }

    private void parkInSafeZone() {
        // Example: Move to a specific parking area
        driveForward(0.5, 1500);
        turn(90); // Turn 90 degrees
    }

    private void pickUpGamePiece() {
        // Example: Move to a game piece and pick it up
        driveForward(0.3, 800); // Approach game piece
        armMotor.setPower(0.5); // Lower arm
        sleep(500);
        clawServo.setPosition(0.0); // Close claw
        armMotor.setPower(0.0); // Stop arm
    }

    private void driveForward(double power, int time) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        sleep(time);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void turn(int angle) {
        // Simplified turn logic
        leftDrive.setPower(0.5);
        rightDrive.setPower(-0.5);
        sleep(500); // Adjust time based on testing
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void updateTaskScores() {
        // Example: Reassess scores based on proximity
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        for (Task task : taskQueue) {
            if (task.name.equals("Pick Up Game Piece")) {
                task.score = (int) (50 - distance); // Higher score for closer game pieces
            }
        }
    }
}
