package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

 public class ArkhamHW {

    /*drive train motors*/
    public DcMotor rightFrontMotor = null;
    public DcMotor rightRearMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor leftRearMotor = null;

    /*Intake and Lift motors*/
    public DcMotor Intake  =  null;
    public DcMotor LiftMotor = null;
    public DcMotor LiftMotor2 = null;

    /*Arkham's servos*/
    public Servo ArmServo = null; //the scoring arm
    public Servo BackServo = null; //recovery servo

    //REV blinkin for the LEDs
    public RevBlinkinLedDriver blinkinLedDriver = null;

    /*Limit Switches*/
    public DigitalChannel topSwitch = null; //Stops the lift when it's all the way up
    public DigitalChannel bottomSwitch = null; ////Stops the lift when it's all the way down

     HardwareMap hwMap = null;

    /**Values for drive train and lift encoders*/

    public static final double COUNTS_PER_MOTOR_REV =  103.0;    // eg: Andy Mark Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 5.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double COUNTS_PER_MOTOR_REV2 =  28;    // eg: Andy Mark Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION2 = 5;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES2 = 1.25;     // For figuring circumference
    public static final double COUNTS_PER_INCH2 = (COUNTS_PER_MOTOR_REV2 * DRIVE_GEAR_REDUCTION2) /
            (WHEEL_DIAMETER_INCHES2 * 3.1415);


    public ArkhamHW() {
    }

    /** Initialize standard Hardware interfaces **/

    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        /* These are the names of each thing plugged into the robot in the confg file in the phone*/
        rightRearMotor = ahwMap.get(DcMotor.class, "rr");
        rightFrontMotor = ahwMap.get(DcMotor.class, "rf");
        leftFrontMotor = ahwMap.get(DcMotor.class, "lf");
        leftRearMotor = ahwMap.get(DcMotor.class, "lr");
        Intake  = ahwMap.get(DcMotor.class,  "i");
        LiftMotor = ahwMap.get(DcMotor.class,"l");
        LiftMotor2 = ahwMap.get(DcMotor.class, "l2");
        ArmServo = ahwMap.servo.get("as");
        BackServo = ahwMap.servo.get("bs");
        blinkinLedDriver = ahwMap.get(RevBlinkinLedDriver.class, "blinkin");
        topSwitch = ahwMap.get(DigitalChannel.class,"t");
        bottomSwitch = ahwMap.get(DigitalChannel.class,"b");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);


        /**The brake allows motors to stop more suddenly instead of drifting to a stop**/
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    /**This creates all the actions that the robot can access later**/
    public void Forward(double speed, double distance){

        leftFrontMotor.setTargetPosition((int) (-distance * COUNTS_PER_INCH));
        leftRearMotor.setTargetPosition((int) (-distance * COUNTS_PER_INCH));
        rightFrontMotor.setTargetPosition((int) (-distance * COUNTS_PER_INCH));
        rightRearMotor.setTargetPosition((int) (-distance * COUNTS_PER_INCH));

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(-speed);
        leftRearMotor.setPower(-speed);
        rightFrontMotor.setPower(-speed);
        rightRearMotor.setPower(-speed);

    }

    public void Reverse(double speed, double distance){

        leftFrontMotor.setTargetPosition((int) (distance* 1 *  COUNTS_PER_INCH));
        leftRearMotor.setTargetPosition((int) (distance* 1 *  COUNTS_PER_INCH));
        rightFrontMotor.setTargetPosition((int) (distance* 1 * COUNTS_PER_INCH));
        rightRearMotor.setTargetPosition((int) (distance* 1 * COUNTS_PER_INCH));

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontMotor.setPower(speed);
        leftRearMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        rightRearMotor.setPower(speed);

    }

    public void TurnAbsolute(double target, double heading){

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double Error   = heading - target;
        double Kp = 0.017;
        double LFPower;
        double LRPower;
        double RFPower;
        double RRPower;
        double EPower;

        if ((Math.abs(Error)) > 2 ){
            LFPower = -Error * Kp;
            LRPower = -Error * Kp;
            RFPower = Error * Kp;
            RRPower = Error * Kp;
            EPower = Error * Kp;

            Range.clip(LFPower,-1,1);
            Range.clip(LRPower,-1,1);
            Range.clip(RFPower,-1,1);
            Range.clip(RRPower,-1,1);
            Range.clip(EPower,-1,1);

            leftFrontMotor.setPower(LFPower);
            leftRearMotor.setPower(LRPower);
            rightFrontMotor.setPower(RFPower);
            rightRearMotor.setPower(RRPower);
        }
        else {
            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);
        }
    }

    public void Align(double target, double heading){

            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double Error = heading - target;
            double Kp = 0.017;
            double LFPower;
            double LRPower;
            double RFPower;
            double RRPower;
            double EPower;

            if ((Math.abs(Error)) > 10 ){
                LFPower = -Error * Kp;
                LRPower = -Error * Kp;
                RFPower = Error * Kp;
                RRPower = Error * Kp;
                EPower = Error * Kp;

                Range.clip(LFPower,-1,1);
                Range.clip(LRPower,-1,1);
                Range.clip(RFPower,-1,1);
                Range.clip(RRPower,-1,1);
                Range.clip(EPower,-1,1);

                leftFrontMotor.setPower(LFPower);
                leftRearMotor.setPower(LRPower);
                rightFrontMotor.setPower(RFPower);
                rightRearMotor.setPower(RRPower);
            }
            else {
                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);
            }
        }


    public void DriveWithPower(double speed){

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontMotor.setPower(speed);
        leftRearMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        rightRearMotor.setPower(speed);
    }

    /**
     Stops everything and resets encoders
     */

    public void Kill(){
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     Checks to see if the motors are running
     and if they're not it will return true
     */

    public boolean IsBusy(){
        if (!leftFrontMotor.isBusy() || !leftRearMotor.isBusy() || !rightFrontMotor.isBusy() || !rightRearMotor.isBusy())
        {
            return (true);
        } else return (false);
    }

    public boolean DriveDone(double distance){
        if ((Math.abs(leftFrontMotor.getCurrentPosition() / COUNTS_PER_INCH) >= distance) ||
                (Math.abs(leftRearMotor.getCurrentPosition() / COUNTS_PER_INCH) >= distance)
                || (Math.abs(rightFrontMotor.getCurrentPosition() / COUNTS_PER_INCH) >= distance) ||
                (Math.abs(rightRearMotor.getCurrentPosition() / COUNTS_PER_INCH) >= distance)
                )
        {
            return (true);
        } else return (false);
    }
}