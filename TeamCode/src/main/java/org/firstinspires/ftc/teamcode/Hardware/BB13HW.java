package org.firstinspires.ftc.teamcode.Hardware;
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

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for our robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 */

public class BB13HW {

    /** Public OpMode Members **/

    public DcMotor  leftDrive = null;
    public DcMotor rightDrive = null;



    public static final double COUNTS_PER_MOTOR_REV =  288;    // eg: Andy Mark Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1.28;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    /** Local OpMode Members. **/

    HardwareMap hwMap = null;

    /** Constructor **/

    public BB13HW() {
    }

    /** Initialize standard Hardware interfaces **/

    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        leftDrive = hwMap.get(DcMotor.class, "ld");
        rightDrive = hwMap.get(DcMotor.class, "rd");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);






        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);


        // Set all motors to zero power

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        leftDrive.setTargetPosition((int) (distance* 53 / 48 * COUNTS_PER_INCH));
        rightDrive.setTargetPosition((int) (distance*53 / 48 *  COUNTS_PER_INCH));
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
    }

    public void Reverse(double speed, double distance){
        leftDrive.setTargetPosition((int) (-distance*53 / 48 *  COUNTS_PER_INCH));
        rightDrive.setTargetPosition((int) (-distance* 53 / 48 * COUNTS_PER_INCH));
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(-speed);
        rightDrive.setPower(-speed);
    }

    public void TurnAbsolute(double target, double heading){

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double Error   = heading - target;
        double Kp = 0.09;
        double LeftPower;
        double RightPower;

        if ((Math.abs(Error)) > 2 ){
            LeftPower = Error * Kp;
            RightPower = -Error * Kp;
            Range.clip(LeftPower,-1,1);
            Range.clip(RightPower,-1,1);

            leftDrive.setPower(LeftPower);
            rightDrive.setPower(RightPower);
        }
        else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

    }

    public void DriveWithPower(double speed){
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
    }

    /**
     Stops everything and resets encoders
     */

    public void Kill(){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     Checks to see if the motors are running
     and if they're not it will return true
     */

    public boolean IsBusy(){
        if (!leftDrive.isBusy() || !rightDrive.isBusy())
        {
            return (true);
        } else return (false);
    }

    public boolean DriveDone(double distance){
        if (Math.abs(leftDrive.getCurrentPosition() / COUNTS_PER_INCH) >= distance
                || Math.abs(rightDrive.getCurrentPosition() / COUNTS_PER_INCH) >= distance)
        {
            return (true);
        } else return (false);
    }

}
