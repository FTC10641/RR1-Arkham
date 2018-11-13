package org.firstinspires.ftc.teamcode.ArkhamTele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


//Disabled
@TeleOp(name="Arkham", group="Linear Opmode")

public class Arkham extends LinearOpMode
{
ArkhamHW robot = new ArkhamHW();
    // Declare OpMode members.

    private boolean toggle = true;
    private boolean toggle2 = false;
    private boolean armtoggle = true;
    private  boolean armtoggle2 = false;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);


        waitForStart();
        if (opModeIsActive()) {


            while (opModeIsActive()) {


                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                if (toggle && gamepad1.left_bumper) {
                    toggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                    if (toggle2) {
                        toggle2 = false;
                        robot.servo.setPosition(0.25);
                       robot.servo2.setPosition(0.75);
                    } else {
                        toggle2 = true;
                        robot.servo.setPosition(1);
                        robot.servo2.setPosition(0);
                    }
                } else if(gamepad1.left_bumper == false) {
                    toggle = true; // Button has been released, so this allows a re-press to activate the code above.
                }


                if (gamepad2.dpad_down && !robot.bottom.getState() == false){
                        robot.Lift.setPower(1);
                        robot.Lift2.setPower(1);
                    }
                    else if(gamepad2.dpad_up && !robot.top.getState() == false){
                        robot.Lift.setPower(-1);
                        robot.Lift2.setPower(-1);}
                    else {
                        robot.Lift.setPower(0);
                        robot.Lift2.setPower(0);
                }

                if (gamepad2.y){robot.armservo.setPosition(0.015);}
                if(gamepad2.right_bumper){robot.armservo.setPosition(0.18);}


                double IntakePower;
                double RRPower;
                double RFPower;
                double LFPower;
                double LRPower;
                double drive = gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                LRPower = Range.clip(drive - turn, -1.0, 1.0);
                LFPower = Range.clip(drive - turn, -1.0, 1.0);
                RRPower = Range.clip(drive + turn, -1.0, 1.0);
                RFPower = Range.clip(drive + turn, -1.0, 1.0);
                IntakePower = Range.clip(-gamepad1.right_trigger + gamepad1.left_trigger,-1.0,1.0);
                // Send calculated power to wheels

                IntakePower = (float) scaleInput(IntakePower);
                LFPower = (float) scaleInput(LFPower);
                LRPower = (float) scaleInput(LRPower);
                RRPower = (float) scaleInput(RRPower);
                RFPower =  (float) scaleInput(RFPower);
                robot.I.setPower(IntakePower);
                robot.LR.setPower(LRPower);
                robot.LF.setPower(LFPower);
                robot.RR.setPower(RRPower);
                robot.RF.setPower(RFPower);
                // Show the elapsed game time and wheel power.

                telemetry.addData("Motors", "left (%.2f), right (%.2f)", LRPower, LFPower, RRPower, RFPower);
                telemetry.update();
            }
        }
    }


    public double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}