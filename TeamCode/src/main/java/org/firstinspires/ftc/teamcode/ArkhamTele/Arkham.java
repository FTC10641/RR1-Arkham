package org.firstinspires.ftc.teamcode.ArkhamTele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.SubSystems.ArkhamHW;


@TeleOp(name="Arkham", group="Linear Opmode") //how the program will appear on the phone

public class Arkham extends LinearOpMode {
ArkhamHW robot = new ArkhamHW();
//allows us to use all of the things stated in the hardware map using robot.



//  stating the pattern names for LED pattern names so they can be used later
    RevBlinkinLedDriver.BlinkinPattern liftGoingUpPattern;
    RevBlinkinLedDriver.BlinkinPattern liftGoingDownPattern;
    RevBlinkinLedDriver.BlinkinPattern neutralPattern;
    RevBlinkinLedDriver.BlinkinPattern liftFullyUpPattern;
    RevBlinkinLedDriver.BlinkinPattern liftFullyDownPattern;

    @Override
    public void runOpMode() {

    /** Adds feedback on the Drive Station phone about the status of the robot. **/
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Start to continue");

        telemetry.update();
        robot.init(hardwareMap);

        //stating the color that the LEDs will have during each pattern
        liftFullyDownPattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
        liftFullyUpPattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
        liftGoingUpPattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
        liftGoingDownPattern = RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE;
        neutralPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;

        waitForStart();
        if (opModeIsActive()) {


            while (opModeIsActive()) {

                /** These are almost all the controls for gamepad one*/

                //these are the controls for our Recovery Servo so we can un-flip the robot
                if (gamepad1.left_bumper){robot.BackServo.setPosition(.5);}
                if (gamepad1.right_bumper){robot.BackServo.setPosition(0);}

                /** These are all the controls for gamepad 2 */

                //these are the controls so our lift can go up and down
                //they are also the controls for the LEDs to change color patter
                if (gamepad2.dpad_down && robot.bottomSwitch.getState()){
                    robot.LiftMotor.setPower(1);
                    robot.LiftMotor2.setPower(-1);
                    robot.blinkinLedDriver.setPattern(liftGoingDownPattern);
                }
                else if(gamepad2.dpad_up && robot.topSwitch.getState()){
                    robot.LiftMotor.setPower(-1);
                    robot.LiftMotor2.setPower(1);
                    robot.blinkinLedDriver.setPattern(liftGoingUpPattern);
                }
                else {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                }
                if(!robot.bottomSwitch.getState()){
                    robot.blinkinLedDriver.setPattern(liftFullyDownPattern);
                }
                if (!robot.topSwitch.getState()){
                    robot.blinkinLedDriver.setPattern(liftFullyUpPattern);
                }

                //these control the Arm that is conncted to the lift (aka our scoring mechanism
                if (gamepad2.a){robot.ArmServo.setPosition(0.015);}
                if(gamepad2.y){robot.ArmServo.setPosition(0.14);}
                if (gamepad2.x){robot.ArmServo.setPosition(.1);}
                /*a is the down position, y is the up position, x is the mid-point*/


                /** Declares and uses motor variables for the Drive Train and Intake **/

                double IntakePower;
                double RRPower;
                double RFPower;
                double LFPower;
                double LRPower;
                double drive = gamepad1.left_stick_y; //allows the robot to drive forward by using the y axis on the left stick
                double turn = gamepad1.right_stick_x; //allows the robot to turn using the x axis on the right stick
                LRPower = Range.clip(drive - turn, -1.0, 1.0);
                LFPower = Range.clip(drive - turn, -1.0, 1.0);
                RRPower = Range.clip(drive + turn, -1.0, 1.0);
                RFPower = Range.clip(drive + turn, -1.0, 1.0);
                IntakePower = Range.clip(-gamepad1.right_trigger + gamepad1.left_trigger,-1.0,1.0); //controls for the Intake motors


                IntakePower = (float) scaleInput(IntakePower);
                LFPower = (float) scaleInput(LFPower);
                LRPower = (float) scaleInput(LRPower);
                RRPower = (float) scaleInput(RRPower);
                RFPower =  (float) scaleInput(RFPower);
                robot.Intake.setPower(IntakePower);
                robot.leftRearMotor.setPower(LRPower);
                robot.leftFrontMotor.setPower(LFPower);
                robot.rightRearMotor.setPower(RRPower);
                robot.rightFrontMotor.setPower(RFPower);
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