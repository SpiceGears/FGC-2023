 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;

 @TeleOp(name="H Drive", group="Linear Opmode")
 @Disabled
 public class Singapore extends LinearOpMode {
 
     // Declare OpMode members.
     private ElapsedTime runtime = new ElapsedTime();
     private DcMotor leftFrontDrive = null;
     private DcMotor leftRearDrive = null;
     private DcMotor rightFrontDrive = null;
     private DcMotor rightRearDrive = null;
     private DcMotor intakeMotor = null;
     private DcMotor elevatorMotor = null;

 
     @Override
     public void runOpMode() {
         telemetry.addData("Status", "Initialized");
         telemetry.update();
 
         leftFrontDrive = hardwareMap.get(DcMotor.class, "lf_d");
         leftRearDrive = hardwareMap.get(DcMotor.class, "lr_d");
         rightFrontDrive = hardwareMap.get(DcMotor.class, "rf_d");
         rightRearDrive = hardwareMap.get(DcMotor.class, "rr_d");
         intakeMotor = hardwareMap.get(DcMotor.class, "in");
         elevatorMotor = hardwareMap.get(DcMotor.class, "el");

         leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
         rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
         intakeMotor.setDirection(DcMotor.Direction.FORWARD);
         elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
 
         // Wait for the game to start (driver presses PLAY)
         waitForStart();
         runtime.reset();
 
         // run until the end of the match (driver presses STOP)
         while (opModeIsActive()) {
 
             // Setup a variable for each drive wheel to save power level for telemetry
             double leftPower;
             double rightPower;
             double intakePower;
             double elevatorPower;
 
             double drive = -gamepad1.left_stick_y;
             double turn  =  gamepad1.right_stick_x;
             intakePower = gamepad1.right_trigger;

             if(gamepad1.dpad_up) {
                 elevatorPower = 1;
             } else if (gamepad1.dpad_down) {
                 elevatorPower = -1;
             } else {
                 elevatorPower = 0;
             }

             leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
             rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
 
             // Send calculated power to wheels
             leftFrontDrive.setPower(leftPower);
             rightFrontDrive.setPower(rightPower);
             intakeMotor.setPower(intakePower);
             elevatorMotor.setPower(elevatorPower);
 
             // Show the elapsed game time and wheel power.
             telemetry.addData("Status", "Run Time: " + runtime.toString());
             telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
             telemetry.addData("Motors", "intake (%.2f)", intakePower);
             telemetry.addData("Motors", "elevator (%.2f)", elevatorPower);
             telemetry.update();

         }
     }
 }
 