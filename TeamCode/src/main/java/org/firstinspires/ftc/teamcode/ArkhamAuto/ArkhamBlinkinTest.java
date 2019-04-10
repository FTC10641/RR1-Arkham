package org.firstinspires.ftc.teamcode.ArkhamTele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SubSystems.ArkhamHW;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;


//Disabled
@TeleOp(name="ArkhamBlinkinTest", group="Linear Opmode")

public class ArkhamBlinkinTest extends LinearOpMode
{
ArkhamHW robot = new ArkhamHW();

    private boolean toggle = true;
    private boolean toggle2 = false;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern liftdownpattern;
    RevBlinkinLedDriver.BlinkinPattern liftuppattern;
    RevBlinkinLedDriver.BlinkinPattern neutralpattern;

    @Override
    public void runOpMode() {

    /** Adds feedback on the Drive Station phone about the status of the robot. **/

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        liftdownpattern = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
        liftuppattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        neutralpattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;

        waitForStart();
        if (opModeIsActive()) {


            while (opModeIsActive()) {


                /** Controls the Lift Motor, allowing it to raise or lower on command **/

                if (gamepad2.dpad_down && !robot.BottomSwitch.getState() == false){
                    robot.LiftMotor.setPower(1);
                    robot.LiftMotor2.setPower(-1);
                }
                else if(gamepad2.dpad_up && !robot.TopSwitch.getState() == false){
                    robot.LiftMotor.setPower(-1);
                    robot.LiftMotor2.setPower(1);
                }
                else {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                }

                if(robot.TopSwitch.getState() && robot.BottomSwitch.getState() == false) {
                    blinkinLedDriver.setPattern(neutralpattern);
                }
                else if(!robot.TopSwitch.getState() == false) {
                    blinkinLedDriver.setPattern(liftuppattern);
                }
                else if(!robot.BottomSwitch.getState() == false) {
                    blinkinLedDriver.setPattern(liftdownpattern);
                }

                /** Controls the Arm Servo, allowing it to move to certain positions on command. **/

                if (gamepad1.left_bumper){robot.BackServo.setPosition(0.25);}
                if (gamepad1.right_bumper){robot.BackServo.setPosition(0);}

                if (gamepad2.a){robot.ArmServo.setPosition(0.015);}
                if(gamepad2.y){robot.ArmServo.setPosition(0.12);}
                if (gamepad2.x){robot.ArmServo.setPosition(.07);}


                /** Declares and uses motor variables for the Drive Train and Intake **/

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
                robot.Intake.setPower(IntakePower);
                robot.LeftRearMotor.setPower(LRPower);
                robot.LeftFrontMotor.setPower(LFPower);
                robot.RightRearMotor.setPower(RRPower);
                robot.RightFrontMotor.setPower(RFPower);
                // Show the elapsed game time and wheel power.

                telemetry.addData("Motors", "left (%.2f), right (%.2f)", LRPower, LFPower, RRPower, RFPower);
                telemetry.update();
            }
        }
    }

    /** This Scale Array function establishes a set of increasing values
        that is used to control the speed of motors and servos. It allows them to exponentially increases their rotation.  **/

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