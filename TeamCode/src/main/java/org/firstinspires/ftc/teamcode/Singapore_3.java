package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Singapore_3", group="Linear Opmode")
// @Disabled
public class Singapore_3 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    // declare hardware
    private DcMotor leftFrontDrive, leftRearDrive = null;
    private DcMotor rightFrontDrive, rightRearDrive = null;
    private DcMotor intakeMotor1, intakeMotor2 = null;
    private DcMotor elevatorMotor1, elevatorMotor2 = null;
    private Servo bucketServo1, bucketServo2 = null;

    // elevator
    DcMotor.RunMode elevatorManualMode = DcMotor.RunMode.RUN_USING_ENCODER;
    DcMotor.RunMode elevatorButtonMode = DcMotor.RunMode.RUN_TO_POSITION;
    final double elevatorButtonModeSpeed = 1;
    final int startElevatorPosition = 0;
    final int minElevatorPosition = 60;
    final int midElevatorPosition = 3400;
    final int maxElevatorPosition = 4700;

    // insert angleToServo (angle from vertical (negative -> to front, positive -> to back) )
    final double bucketRestPosition = angleToServo(132); // 0.83 angleToServo(-90)
    final double bucketFlippedPosition = angleToServo(15); // 0.59 angleToServo(25)
    final int bucketFlipThreshold = 3000; // if over this amount of ticks, can flip bucket

    // setup a variable for setting power and telemetry
    double leftPower, rightPower;
    double intakePower;
    private double averageRobotSpeed = 0;
    private double speedNow, speedSum, speedCount = 0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // map hardware to port names
        mapHardware();

        // setup hardware properties
        setupHardware();

        // wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run once after pressing start
        // setups motors and positions when match starts
        setupOnMatchStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // controls drivetrain
            double drive =  gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;


            // controls intake
            if(gamepad1.left_trigger > gamepad1.right_trigger) {
                intakePower = gamepad1.left_trigger;
                if(intakePower > 0.8) { intakePower = 1; } // make intake spin full throttle at 80% input

            } else if(gamepad1.left_trigger < gamepad1.right_trigger) {
                intakePower = -gamepad1.right_trigger;
                if(intakePower < -0.8) { intakePower = -1; } // make intake spin full throttle at 80% input

            } else { intakePower = 0; }


            // switches elevator modes
            if(gamepad2.left_bumper) {
                setElevatorMode(elevatorManualMode);
            }
            if(gamepad2.right_bumper) {
                setElevatorMode(elevatorButtonMode);
            }

            // operates manual elevator mode
            if(elevatorMotor1.getMode() == elevatorManualMode) {

                setElevatorSpeed(-gamepad2.left_stick_y); // set elevator speed to left joystick
                setElevatorPosition(elevatorMotor1.getCurrentPosition()); // set elevator to stay after switching to button mode
            }

            // operates button elevator mode
            if(elevatorMotor1.getMode() == elevatorButtonMode) {

                setElevatorSpeed(elevatorButtonModeSpeed); // set elevator to move at given speed when in button mode

                // set chosen position
                if(gamepad2.a){
                    setElevatorPosition(minElevatorPosition);
                }
                if (gamepad2.x) {
                    setElevatorPosition(midElevatorPosition);
                }
                if (gamepad2.y) {
                    setElevatorPosition(maxElevatorPosition);
                }
            }


            // controls bucket
            if(gamepad2.b) {
                setBucketPosition(bucketFlippedPosition);
            } else {
                setBucketPosition(bucketRestPosition);
            }

            // bucket safety feature on button mode
            if(elevatorMotor1.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                if(elevatorMotor1.getCurrentPosition() < bucketFlipThreshold) { // if elevator below bucket flip height threshold
                    if(bucketServo1.getPosition() >= 1.1 * bucketRestPosition) { // if bucket position isn't in acceptable position
                        // stop elevator unless bucket is good to go
                        setBucketPosition(bucketRestPosition);
                        setElevatorSpeed(0);
                    }
                } else { // if elevator over bucket flip height threshold and bucket is flipped
                    // go like nothing happened
                    setElevatorSpeed(elevatorButtonModeSpeed);
                }
            }


            // calculate for telemetry
            speedNow = Math.abs(leftPower) + Math.abs(rightPower) / 2;
            speedCount++;
            speedSum += speedNow;
            averageRobotSpeed = speedSum / speedCount;

            // sends calculated power to wheels
            sendPower();

            // sends info about match and robot to driver station
            sendTelemetry();

        }

    }


    void setBucketPosition(double position) {
        bucketServo1.setPosition(position);
        bucketServo2.setPosition(position);
    }

    void setElevatorPosition(int position) {
        elevatorMotor1.setTargetPosition(position);
        elevatorMotor2.setTargetPosition(position);
    }
    void setElevatorSpeed(double power) {
        elevatorMotor1.setPower(power);
        elevatorMotor2.setPower(power);
    }
    void setElevatorMode(DcMotor.RunMode runmode) {
        elevatorMotor1.setMode(runmode);
        elevatorMotor2.setMode(runmode);
    }

    // maps hardware to ports
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

    // setups robot hardware
    void setupHardware() {
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
    }

    // setups motors and positions when match starts
    void setupOnMatchStart() {
        if(opModeIsActive()) {
            setElevatorPosition(startElevatorPosition);
            elevatorMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setElevatorSpeed(elevatorButtonModeSpeed);
            setBucketPosition(bucketRestPosition);
        }
    }

    // sends info about robot to driver station
    void sendTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Motors", "intake (%.2f)", intakePower);
        telemetry.addData("avg robot speed", averageRobotSpeed);
        telemetry.addData("Elevator1EncoderValue", elevatorMotor1.getCurrentPosition());
        telemetry.addData("Elevator2EncoderValue", elevatorMotor2.getCurrentPosition());
        telemetry.addData("Elevator1TargetPosition", elevatorMotor1.getTargetPosition());
        telemetry.addData("Elevator2TargetPosition", elevatorMotor2.getTargetPosition());
        telemetry.addData("ElevatorMode", elevatorMotor1.getMode());
        telemetry.addData("ServoGetTargetPosition", bucketServo1.getPosition());
        telemetry.update();
    }

    // sends calculated power to motors/servos
    void sendPower() {
        leftFrontDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        leftRearDrive.setPower(leftPower);
        rightRearDrive.setPower(rightPower);
        intakeMotor1.setPower(intakePower);
        intakeMotor2.setPower(intakePower);
    }

    // calculates angle from vertical (negative -> front, positive -> back)
    double angleToServo(double angle) {
        return ((135 + angle )/ 270);
        // returned 0 -> 135degrees back from vertical
        // returned 1 -> 135degrees front from vertical
    }
}
