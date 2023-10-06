package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// import org.firstinspires.ftc.robotcore.external.ClassFactory;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@TeleOp(name="Singapore_experimental", group="Linear Opmode")
// @Disabled
public class Singapore_experimental extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // private static final String VUFORIA_KEY = "ARl//vr/////AAABmTtsXXGN4kcFkhhcsszd9P6M4/IHmiJcHYW/o7r7xpmYm8DUrEK1VMWutn9OyxB74EATeD0NetM+Dp5ohcXx1YpHw//1hzHCQEtTNg8fukwHRwoljRnoErbcN3SSNsNvOKVO+/R2OuetPMQORH1bB0cbJhZ+iJORrbVlJFzSCrnU0s1dZi053cdF0OIASA/AuF62EQ+pCSNzB/7Gpy+SdANNEM768ZaCuxYKAKcfSzPE1czBQ6LY9kxHBjDE3XwB+y1pZSF/pjALepudxgXe4FXkP+sPycecg2+/YGnOeJ49aALvVGbEhVy7sSA6rID9VAe/Lga2S+NKE69qmNPIoWopBZO6WhGhlCcd+OzTAbav";

    // VuforiaLocalizer vuforia    = null;
    // OpenGLMatrix targetPose     = null;
    // String targetName           = "";

    // declare hardware
    private DcMotor leftFrontDrive, leftRearDrive = null;
    private DcMotor rightFrontDrive, rightRearDrive = null;
    private DcMotor intakeMotor1, intakeMotor2 = null;
    private DcMotor elevatorMotor1, elevatorMotor2 = null;
    private Servo bucketServo1, bucketServo2 = null;
    private Servo clawServo1 = null;
    //    private Servo clawServo2 = null;
    private Servo sorterServo1, sorterServo2 = null;


    // elevator
    DcMotor.RunMode elevatorManualMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    DcMotor.RunMode elevatorButtonMode = DcMotor.RunMode.RUN_TO_POSITION;
    final double elevatorButtonModeSpeed = 1;
    final int startElevatorPosition = 0;
    final int minElevatorPosition = 60;
    final int midElevatorPosition = 3400;
    final int maxElevatorPosition = 4700;

    // bucket servo
    // insert angleToServo (angle from vertical (negative -> to front, positive -> to back) )
    final double bucketRestPosition = angleToServo(135); // 0.83 angleToServo(-90)
    final double bucketFlippedPosition = angleToServo(15); // 0.59 angleToServo(25)
    final int bucketFlipThreshold = 3000; // if over this amount of ticks, can flip bucket

    // claw servo
    final double clawStartPosition = angleToServo(70);
    final double clawOpenPosition = angleToServo(-120);

    // intake servo
    final double sorterStartPosition = angleToServo(90);
    final double sorterOpenPosition = angleToServo(-100);

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


            // controls claw
            if(gamepad1.a) { setClawPosition(clawOpenPosition); }
            if(gamepad1.y) { setClawPosition(clawStartPosition); }


            // controls sorter
            if(gamepad1.dpad_down) { setSorterPosition(sorterOpenPosition); }
            if(gamepad1.dpad_up) { setSorterPosition(sorterStartPosition); }


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

        // keeps elevator braked when program is off
        setElevatorBrake();

    }

    void setClawPosition(double rawPosition) {
        clawServo1.setPosition(rawPosition);
//        clawServo2.setPosition(rawPosition);
    }
    void setSorterPosition(double rawPosition) {
        sorterServo1.setPosition(rawPosition);
        sorterServo2.setPosition(rawPosition);
    }
    void setBucketPosition(double rawPosition) {
        bucketServo1.setPosition(rawPosition);
        bucketServo2.setPosition(rawPosition);
    }
    void setElevatorPosition(int rawPosition) {
        elevatorMotor1.setTargetPosition(rawPosition);
        elevatorMotor2.setTargetPosition(rawPosition);
    }
    void setElevatorSpeed(double power) {
        elevatorMotor1.setPower(power);
        elevatorMotor2.setPower(power);
    }
    void setElevatorMode(DcMotor.RunMode runmode) {
        elevatorMotor1.setMode(runmode);
        elevatorMotor2.setMode(runmode);
    }
    void setElevatorBrake() {
        elevatorMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    String getBucketPositionName() {
        String positionName = "unspecified";
        if(bucketServo1.getPosition() == bucketRestPosition) { positionName = "rest"; }
        if(bucketServo1.getPosition() == bucketFlippedPosition) { positionName = "flipped"; }
        return positionName;
    }
    String getClawPositionName() {
        String positionName = "unspecified";
        if(clawServo1.getPosition() == clawOpenPosition) { positionName = "down"; }
        if(clawServo1.getPosition() == clawStartPosition) { positionName = "up"; }
        return positionName;
    }
    String getSorterPositionName() {
        String positionName = "unspecified";
        if(sorterServo1.getPosition() == sorterOpenPosition) { positionName = "down"; }
        if(sorterServo1.getPosition() == sorterStartPosition) { positionName = "up"; }
        return positionName;
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
        clawServo1 = hardwareMap.get(Servo.class, "c_s1");
//        clawServo2 = hardwareMap.get(Servo.class, "c_s2");
        sorterServo1 = hardwareMap.get(Servo.class, "s_s1");
        sorterServo2 = hardwareMap.get(Servo.class, "s_s2");
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
        clawServo1.setDirection(Servo.Direction.FORWARD);
//        clawServo2.setDirection(Servo.Direction.REVERSE);
        sorterServo1.setDirection(Servo.Direction.FORWARD);
        sorterServo2.setDirection(Servo.Direction.REVERSE);

        elevatorMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setClawPosition(clawStartPosition);
        setSorterPosition(sorterStartPosition);

    }

    // setups motors and positions when match starts
    void setupOnMatchStart() {
        if(opModeIsActive()) {
            setElevatorPosition(startElevatorPosition);
            elevatorMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setElevatorSpeed(elevatorButtonModeSpeed);
            setBucketPosition(bucketRestPosition);
            setClawPosition(clawStartPosition);
            setSorterPosition(sorterOpenPosition);
        }
    }

    // sends info about robot to driver station
    void sendTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Motors", "intake (%.2f)", intakePower);
        telemetry.addData("Elevator position", elevatorMotor1.getCurrentPosition());
        telemetry.addData("Elevator target", elevatorMotor1.getTargetPosition());
        telemetry.addData("Elevator mode", elevatorMotor1.getMode());
        telemetry.addData("Bucket position", getBucketPositionName().toUpperCase());
        telemetry.addData("Claw position", getClawPositionName().toUpperCase());
        telemetry.addData("Sorter position", getSorterPositionName().toUpperCase());
        telemetry.addData("avg robot speed", averageRobotSpeed);
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
