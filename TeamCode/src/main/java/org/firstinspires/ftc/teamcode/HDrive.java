 package org.firstinspires.ftc.robotcontroller.external.samples;

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.util.Range;
 
 @TeleOp(name="H Drive", group="Linear Opmode")
 @Disabled
 public class HDrive extends LinearOpMode {
 
     // Declare OpMode members.
     private ElapsedTime runtime = new ElapsedTime();
     private DcMotor leftDrive = null;
     private DcMotor rightDrive = null;
     private DcMotor hDrive = null;

 
     @Override
     public void runOpMode() {
         telemetry.addData("Status", "Initialized");
         telemetry.update();
 
         leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
         rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
         hDrive = hardwareMap.get(DcMotor.class, "h_drive");

         leftDrive.setDirection(DcMotor.Direction.REVERSE);
         rightDrive.setDirection(DcMotor.Direction.FORWARD);
         hDrive.setDirection(DcMotor.Direction.FORWARD);
 
         // Wait for the game to start (driver presses PLAY)
         waitForStart();
         runtime.reset();
 
         // run until the end of the match (driver presses STOP)
         while (opModeIsActive()) {
 
             // Setup a variable for each drive wheel to save power level for telemetry
             double leftPower;
             double rightPower;
             double hPower;
 
             double drive = -gamepad1.left_stick_y;
             double turn  =  gamepad1.right_stick_x;
             double h  =  gamepad1.left_stick_x;

             leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
             rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
             hPower   = Range.clip(hPower, -1.0, 1.0) ;
 
             // Send calculated power to wheels
             leftDrive.setPower(leftPower);
             rightDrive.setPower(rightPower);
 
             // Show the elapsed game time and wheel power.
             telemetry.addData("Status", "Run Time: " + runtime.toString());
             telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
             telemetry.addData("Motors", "h (%.2f)", hPower);
             telemetry.update();
             
         }
     }
 }
 