package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Singapore_2", group="Linear Opmode")
// @Disabled
public class Singapore_2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //declare hardware
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor intakeMotor1 = null;
    private DcMotor intakeMotor2 = null;
    private DcMotor elevatorMotor1 = null;
    private DcMotor elevatorMotor2 = null;
    private Servo bucketServo1 = null;
    private Servo bucketServo2 = null;

    private double averageRobotSpeed = 0;
    private double speedNow = 0;
    private double speedSum = 0;
    private double speedCount = 0;

    double elevatorMoveSpeed = 1;
    int startElevatorPosition = 0;
    int maxElevatorPosition = 4700;
    int minElevatorPosition = 60;
    int midElevatorPosition = 3400;

    double bucketFlipSpeed = 0.6;
    int bucketRestPosition = 0;
    int bucketFlipPosition = 100;
    int bucketFlipTreshold = 3000; // if over this amount of ticks, can flip bucket

    // insert angleToServo(angle from vertical)
    double servoStartPosition = angleToServo(-90);
    double servoFlippedPosition = angleToServo(25);

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;
    double intakePower;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // map hardware to port names
        mapHardware();


        // set hardware properties
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor1.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor2.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor1.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor2.setDirection(DcMotor.Direction.REVERSE);
        bucketServo1.setDirection(Servo.Direction.FORWARD);
        bucketServo2.setDirection(Servo.Direction.REVERSE);

        elevatorMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        elevatorMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run once after pressing start
        if(opModeIsActive()) {
            elevatorMotor1.setTargetPosition(startElevatorPosition);
            elevatorMotor2.setTargetPosition(startElevatorPosition);
            elevatorMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor1.setPower(elevatorMoveSpeed);
            elevatorMotor2.setPower(elevatorMoveSpeed);
            bucketServo1.setPosition(servoStartPosition);
            bucketServo2.setPosition(servoStartPosition);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double drive = gamepad1.left_stick_y;
            double turn  =  -gamepad1.right_stick_x;

            if(gamepad1.right_trigger > gamepad1.left_trigger) {
                intakePower = gamepad1.right_trigger;
            } else if(gamepad1.right_trigger < gamepad1.left_trigger) {
                intakePower = -gamepad1.left_trigger;
            } else {
                intakePower = 0;
            }

            if(gamepad1.b) {
                bucketServo1.setPosition(servoFlippedPosition);
                bucketServo2.setPosition(servoFlippedPosition);
            } else {
                bucketServo1.setPosition(servoStartPosition);
                bucketServo2.setPosition(servoStartPosition);
            }


            if(gamepad1.a){
                elevatorMotor1.setTargetPosition(minElevatorPosition);
                elevatorMotor2.setTargetPosition(minElevatorPosition);
            }
            if (gamepad1.x) {
                elevatorMotor1.setTargetPosition(midElevatorPosition);
                elevatorMotor2.setTargetPosition(midElevatorPosition);
            }
            if (gamepad1.y) {
                elevatorMotor1.setTargetPosition(maxElevatorPosition);
                elevatorMotor2.setTargetPosition(maxElevatorPosition);
            }

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            speedNow = Math.abs(leftPower) + Math.abs(rightPower) / 2;
            speedCount++;
            speedSum += speedNow;
            averageRobotSpeed = speedSum / speedCount;


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            leftRearDrive.setPower(leftPower);
            rightRearDrive.setPower(rightPower);
            intakeMotor1.setPower(intakePower);
            intakeMotor2.setPower(intakePower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Motors", "intake (%.2f)", intakePower);
            telemetry.addData("avg robot speed", averageRobotSpeed);
            telemetry.addData("Elevator1EncoderValue", elevatorMotor1.getCurrentPosition());
            telemetry.addData("Elevator2EncoderValue", elevatorMotor2.getCurrentPosition());
            telemetry.addData("Elevator1TargetPosition", elevatorMotor1.getTargetPosition());
            telemetry.addData("Elevator2TargetPosition", elevatorMotor2.getTargetPosition());
            telemetry.addData("ElevatorMode", elevatorMotor1.getMode());

            telemetry.update();


        }

    }

    //map hardware to ports
    void mapHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf_d");
        leftRearDrive = hardwareMap.get(DcMotor.class, "lr_d");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf_d");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rr_d");
        elevatorMotor1 = hardwareMap.get(DcMotor.class, "el");
        elevatorMotor2 = hardwareMap.get(DcMotor.class, "el1");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "in");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "in1");
        bucketServo1 = hardwareMap.get(Servo.class, "b_s1");
        bucketServo2 = hardwareMap.get(Servo.class, "b_s2");
    }

    // calculates angle from vertical (positive -> front, negative -> back)
    double angleToServo(double angle) {
        return angle = ((135 + angle )/ 270);
    }
}
