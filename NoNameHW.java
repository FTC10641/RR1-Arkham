package org.firstinspires.ftc.teamcode.Hardware;
import android.text.util.Rfc822Token;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/** Copy of BB13HW, adding specifications for all four motors. **/

public class NoNameHW {

    public DcMotor RF = null;
    public DcMotor RR = null;
    public DcMotor LF = null;
    public DcMotor LR = null;


    public static final double COUNTS_PER_MOTOR_REV =  103.0;    // eg: Andy Mark Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 5.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /** Local OpMode Members. **/

    HardwareMap hwMap = null;

    /** Constructor **/

    public NoNameHW() {
    }

    /** Initialize standard Hardware interfaces **/


    /** Initialize standard Hardware interfaces **/

    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        RF = hwMap.get(DcMotor.class, "rf");
        RR = hwMap.get(DcMotor.class, "rr");
        LF = hwMap.get(DcMotor.class, "lf");
        LR = hwMap.get(DcMotor.class, "lr");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LF.setDirection(DcMotor.Direction.FORWARD);
        LR.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        RF.setPower(0);
        RR.setPower(0);
        LF.setPower(0);
        LR.setPower(0);


        // Set all motors to zero power

        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     This method scales the joystick input so for low joystick values, the
     scaled value is less than linear.  This is to make it easier to drive
     the robot more precisely at slower speeds.
     */

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

    public void Forward(double speed, double distance){
        LF.setTargetPosition((int) (distance* 53 / 48 * COUNTS_PER_INCH));
        LR.setTargetPosition((int) (distance* 53 / 48 * COUNTS_PER_INCH));
        RF.setTargetPosition((int) (distance* 53 / 48 *  COUNTS_PER_INCH));
        RR.setTargetPosition((int) (distance* 53 / 48 *  COUNTS_PER_INCH));
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setPower(speed);
        LR.setPower(speed);
        RF.setPower(speed);
        RR.setPower(speed);
    }

    public void Reverse(double speed, double distance){
        LF.setTargetPosition((int) (-distance*53 / 48 *  COUNTS_PER_INCH));
        LR.setTargetPosition((int) (-distance*53 / 48 *  COUNTS_PER_INCH));
        RF.setTargetPosition((int) (-distance* 53 / 48 * COUNTS_PER_INCH));
        RR.setTargetPosition((int) (-distance* 53 / 48 * COUNTS_PER_INCH));
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setPower(-speed);
        LR.setPower(-speed);
        RF.setPower(-speed);
        RR.setPower(-speed);

    }

    public void TurnAbsolute(double target, double heading){

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double Error   = heading - target;
        double Kp = 0.09;
        double LFPower;
        double LRPower;
        double RFPower;
        double RRPower;

        if ((Math.abs(Error)) > 2 ){
            LFPower = Error * Kp;
            LRPower = Error * Kp;
            RFPower = -Error * Kp;
            RRPower = -Error * Kp;
            Range.clip(LFPower,-1,1);
            Range.clip(LRPower,-1,1);
            Range.clip(RFPower,-1,1);
            Range.clip(RRPower,-1,1);

            LF.setPower(LFPower);
            LR.setPower(LRPower);
            RF.setPower(RFPower);
            RR.setPower(RRPower);
        }
        else {
            LF.setPower(0);
            LR.setPower(0);
            RF.setPower(0);
            RR.setPower(0);
        }

    }

    public void DriveWithPower(double speed){
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setPower(speed);
        LR.setPower(speed);
        RF.setPower(speed);
        RR.setPower(speed);
    }

    /**
     Stops everything and resets encoders
     */

    public void Kill(){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     Checks to see if the motors are running
     and if they're not it will return true
     */

    public boolean IsBusy(){
        if (!LF.isBusy() || !LR.isBusy() || !RF.isBusy() || !RR.isBusy())
        {
            return (true);
        } else return (false);
    }

    public boolean DriveDone(double distance){
        if ((Math.abs(LF.getCurrentPosition() / COUNTS_PER_INCH) >= distance) ||
                (Math.abs(LR.getCurrentPosition() / COUNTS_PER_INCH) >= distance)
                || (Math.abs(RF.getCurrentPosition() / COUNTS_PER_INCH) >= distance) ||
                (Math.abs(RR.getCurrentPosition() / COUNTS_PER_INCH) >= distance))
        {
            return (true);
        } else return (false);
    }







}
