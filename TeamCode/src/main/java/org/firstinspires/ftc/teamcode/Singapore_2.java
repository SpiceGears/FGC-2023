package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Singapore_2", group="Linear Opmode")
// @Disabled
public class Singapore_2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor intakeMotor1 = null;
    private DcMotor intakeMotor2 = null;
    private DcMotor elevatorMotor1 = null;
    private DcMotor elevatorMotor2 = null;
    //private DcMotor bucketMotor = null;
    private Servo bucketServo = null;

    private double averageRobotSpeed = 0;
    private double speedNow = 0;
    private double speedSum = 0;
    private double speedCount = 0;

    double elevatorMoveSpeed = 0.6;
    int maxElevatorPosition = 7200;
    int minElevatorPosition = 60;
    int midElevatorPosition = 3600;

    double bucketFlipSpeed = 0.6;
    int bucketRestPosition = 0;
    int bucketFlipPosition = 100;
    int bucketFlipTreshold = 3000; // if over this amount of ticks, can flip bucket


    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;
    double intakePower;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf_d");
        leftRearDrive = hardwareMap.get(DcMotor.class, "lr_d");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf_d");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rr_d");
        elevatorMotor1 = hardwareMap.get(DcMotor.class, "el");
        elevatorMotor2 = hardwareMap.get(DcMotor.class, "el1");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "in");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "in1");
        //  bucketMotor = hardwareMap.get(DcMotor.class, "bu");
        bucketServo = hardwareMap.get(Servo.class, "b_s");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor1.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor2.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor1.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor2.setDirection(DcMotor.Direction.REVERSE);
        //bucketMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //bucketMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //bucketMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

//        elevatorMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
//        elevatorMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        // run once after pressing start
        if(opModeIsActive()) {
            elevatorMotor1.setTargetPosition(minElevatorPosition);
            elevatorMotor2.setTargetPosition(minElevatorPosition);
            //  bucketMotor.setTargetPosition(bucketRestPosition);
            elevatorMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //bucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor1.setPower(elevatorMoveSpeed);
            elevatorMotor2.setPower(0.6);
            //bucketMotor.setPower(0.6);

            bucketServo.setPosition(100);
            bucketServo.setDirection(Servo.Direction.FORWARD);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            if(gamepad1.right_trigger > gamepad1.left_trigger) {
                intakePower = gamepad1.right_trigger;
            } else if(gamepad1.right_trigger < gamepad1.left_trigger) {
                intakePower = -gamepad1.left_trigger;
            } else {
                intakePower = 0;
            }

            if(gamepad1.b) {
                bucketServo.setPosition(0);
            } else {
                bucketServo.setPosition(100);
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
            //if (gamepad1.left_bumper && (elevatorMotor1.getCurrentPosition() > bucketFlipTreshold) ) {
            //bucketMotor.setTargetPosition(bucketFlipPosition);
            //}
            //if (gamepad1.right_bumper) {
            //  bucketMotor.setTargetPosition(bucketRestPosition);
            //}

            //if (elevatorMotor1.getTargetPosition() < bucketFlipTreshold) {
            //   bucketMotor.setTargetPosition(bucketRestPosition);
            //}
            //if (elevatorMotor1.getCurrentPosition() < bucketFlipTreshold && bucketMotor.getCurrentPosition() > 10+bucketRestPosition) {
            //elevatorMotor1.setPower(0);
            //  elevatorMotor2.setPower(0);
            //} else {
            //elevatorMotor1.setPower(elevatorMoveSpeed);
            //  elevatorMotor2.setPower(elevatorMoveSpeed);
            //}



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
            telemetry.addData("ElevatorEncoderValue", elevatorMotor1.getCurrentPosition()+elevatorMotor2.getCurrentPosition()/2);
            telemetry.addData("ElevatorTargetPosition", elevatorMotor1.getTargetPosition());
            telemetry.addData("ElevatorMode", elevatorMotor1.getMode());

            telemetry.update();

        }
    }
}
 