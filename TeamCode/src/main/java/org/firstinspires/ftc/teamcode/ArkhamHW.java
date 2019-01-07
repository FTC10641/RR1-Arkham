package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class ArkhamHW {


    public DcMotor RightFrontMotor = null;
    public DcMotor RightRearMotor = null;
    public DcMotor LeftFrontMotor = null;
    public DcMotor LeftRearMotor = null;
    public DcMotor Intake  =  null;
    public DcMotor LiftMotor = null;
    public DcMotor LiftMotor2 = null;
    public Servo LeftSorterServo = null;
    public Servo RightSorterServo = null;
    public Servo ArmServo = null;
    public DigitalChannel TopSwitch = null;
    public DigitalChannel BottomSwitch = null;

/**Values for drive train and lift encoders*/

    public static final double COUNTS_PER_MOTOR_REV =  103.0;    // eg: Andy Mark Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 5.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double COUNTS_PER_MOTOR_REV2 =  537.6;    // eg: Andy Mark Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION2 = 1.75;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES2 = 2.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH2 = (COUNTS_PER_MOTOR_REV2 * DRIVE_GEAR_REDUCTION2) /
            (WHEEL_DIAMETER_INCHES2 * 3.1415);

    /** Local OpMode Members. */


    HardwareMap hwMap = null;

    /** Constructor */

    public ArkhamHW() {
    }

    /** Initialize standard Hardware interfaces **/

    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;


        LeftSorterServo = ahwMap.servo.get("ls");
        RightSorterServo = ahwMap.servo.get("rs");
        ArmServo = ahwMap.servo.get("as");




        /** Initialize the hardware variables. Note that the strings used here as parameters
        to 'get' must correspond to the names assigned during the robot configuration**/
        RightRearMotor = ahwMap.get(DcMotor.class, "rr");
        RightFrontMotor = ahwMap.get(DcMotor.class, "rf");
        LeftFrontMotor = ahwMap.get(DcMotor.class, "lf");
        LeftRearMotor = ahwMap.get(DcMotor.class, "lr");
        Intake  = ahwMap.get(DcMotor.class,  "i");
        LiftMotor = ahwMap.get(DcMotor.class,"l");
        LiftMotor2 = ahwMap.get(DcMotor.class, "l2");
        TopSwitch = ahwMap.get(DigitalChannel.class,"t");
        BottomSwitch = ahwMap.get(DigitalChannel.class,"b");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LeftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        RightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        RightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        /**The brake allows motors to stop more suddenly instead of drifting to a stop**/
        LeftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        LeftFrontMotor.setTargetPosition((int) (-distance * COUNTS_PER_INCH));
        LeftRearMotor.setTargetPosition((int) (-distance * COUNTS_PER_INCH));
        RightFrontMotor.setTargetPosition((int) (-distance * COUNTS_PER_INCH));
        RightRearMotor.setTargetPosition((int) (-distance * COUNTS_PER_INCH));
        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFrontMotor.setPower(-speed);
        LeftRearMotor.setPower(-speed);
        RightFrontMotor.setPower(-speed);
        RightRearMotor.setPower(-speed);
    }

    public void Reverse(double speed, double distance){
        LeftFrontMotor.setTargetPosition((int) (distance* 1 *  COUNTS_PER_INCH));
        LeftRearMotor.setTargetPosition((int) (distance* 1 *  COUNTS_PER_INCH));
        RightFrontMotor.setTargetPosition((int) (distance* 1 * COUNTS_PER_INCH));
        RightRearMotor.setTargetPosition((int) (distance* 1 * COUNTS_PER_INCH));
        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFrontMotor.setPower(speed);
        LeftRearMotor.setPower(speed);
        RightFrontMotor.setPower(speed);
        RightRearMotor.setPower(speed);
    }

    public void TurnAbsolute(double target, double heading){

        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double Error   = heading - target;
        double Kp = 0.02;
        double LFPower;
        double LRPower;
        double RFPower;
        double RRPower;

        if ((Math.abs(Error)) > 2 ){
            LFPower = -Error * Kp;
            LRPower = -Error * Kp;
            RFPower = Error * Kp;
            RRPower = Error * Kp;
            Range.clip(LFPower,-1,1);
            Range.clip(LRPower,-1,1);
            Range.clip(RFPower,-1,1);
            Range.clip(RRPower,-1,1);

            LeftFrontMotor.setPower(LFPower);
            LeftRearMotor.setPower(LRPower);
            RightFrontMotor.setPower(RFPower);
            RightRearMotor.setPower(RRPower);
        }
        else {
            LeftFrontMotor.setPower(0);
            LeftRearMotor.setPower(0);
            RightFrontMotor.setPower(0);
            RightRearMotor.setPower(0);
        }
    }

    public void Align(double target, double heading){

            LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double Error   = heading - target;
            double Kp = 0.001;
            double LFPower;
            double LRPower;
            double RFPower;
            double RRPower;

            if ((Math.abs(Error)) > 10 ){
                LFPower = -Error * Kp;
                LRPower = -Error * Kp;
                RFPower = Error * Kp;
                RRPower = Error * Kp;
                Range.clip(LFPower,-1,1);
                Range.clip(LRPower,-1,1);
                Range.clip(RFPower,-1,1);
                Range.clip(RRPower,-1,1);

                LeftFrontMotor.setPower(LFPower);
                LeftRearMotor.setPower(LRPower);
                RightFrontMotor.setPower(RFPower);
                RightRearMotor.setPower(RRPower);
            }
            else {
                LeftFrontMotor.setPower(0);
                LeftRearMotor.setPower(0);
                RightFrontMotor.setPower(0);
                RightRearMotor.setPower(0);
            }
        }


    public void DriveWithPower(double speed){
        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFrontMotor.setPower(speed);
        LeftRearMotor.setPower(speed);
        RightFrontMotor.setPower(speed);
        RightRearMotor.setPower(speed);
    }

    /**
     Stops everything and resets encoders
     */

    public void Kill(){
        LeftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     Checks to see if the motors are running
     and if they're not it will return true
     */

    public boolean IsBusy(){
        if (!LeftFrontMotor.isBusy() || !LeftRearMotor.isBusy() || !RightFrontMotor.isBusy() || !RightRearMotor.isBusy())
        {
            return (true);
        } else return (false);
    }

    public boolean DriveDone(double distance){
        if ((Math.abs(LeftFrontMotor.getCurrentPosition() / COUNTS_PER_INCH) >= distance) ||
                (Math.abs(LeftRearMotor.getCurrentPosition() / COUNTS_PER_INCH) >= distance)
                || (Math.abs(RightFrontMotor.getCurrentPosition() / COUNTS_PER_INCH) >= distance) ||
                (Math.abs(RightRearMotor.getCurrentPosition() / COUNTS_PER_INCH) >= distance))
        {
            return (true);
        } else return (false);
    }





}