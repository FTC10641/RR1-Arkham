package org.firstinspires.ftc.teamcode.ArkhamTele;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


//Disabled
@TeleOp(name="Arkham", group="Linear Opmode")

public class Arkham extends LinearOpMode
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor RF = null;
    private DcMotor RR = null;
    private DcMotor LR = null;
    private DcMotor LF = null;
    private DcMotor I  =  null;
    private DcMotor Lift = null;
    private DcMotor Lift2 = null;
    private Servo servo = null;
    private Servo servo2 = null;
    private Servo armservo = null;
    private boolean toggle = true;
    private boolean toggle2 = false;
    private boolean armtoggle = true;
    private  boolean armtoggle2 = false;

    public static final double COUNTS_PER_MOTOR_REV =  537.6;    // eg: Andy Mark Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1.75;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 2.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        servo = hardwareMap.servo.get("ls");
        servo2 = hardwareMap.servo.get("rs");
        armservo = hardwareMap.servo.get("as");

        servo.setPosition(1);
        servo2.setPosition(0);
        armservo.setPosition(0);


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        RR = hardwareMap.get(DcMotor.class, "RR");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");
        I  = hardwareMap.get(DcMotor.class,  "i");
        Lift = hardwareMap.get(DcMotor.class,"L");
        Lift2 = hardwareMap.get(DcMotor.class, "L2");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LF.setDirection(DcMotor.Direction.FORWARD);
        LR.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);

        //The brake allows motors to stop more suddenly instead of drifting to a stop
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        I.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);





        waitForStart();
        if (opModeIsActive()) {

            runtime.reset();

            while (opModeIsActive()) {
                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                if (toggle && gamepad1.left_bumper) {
                    toggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                    if (toggle2) {
                        toggle2 = false;
                        servo.setPosition(0.25);
                        servo2.setPosition(0.75);
                    } else {
                        toggle2 = true;
                        servo.setPosition(1);
                        servo2.setPosition(0);
                    }
                } else if(gamepad1.left_bumper == false) {
                    toggle = true; // Button has been released, so this allows a re-press to activate the code above.
                }


                if (armtoggle && gamepad1.right_bumper) {
                    armtoggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                    if (armtoggle2) {
                        armtoggle2 = false;
                    } else {
                        armtoggle2 = true;
                    }
                } else if(gamepad1.right_bumper == false) {
                    armtoggle = true; // Button has been released, so this allows a re-press to activate the code above.
                }

                if (armtoggle2 && (Math.abs(Lift.getCurrentPosition()/COUNTS_PER_INCH) < 12) && (Math.abs(Lift2.getCurrentPosition()/COUNTS_PER_INCH)<12) ){
                   Lift.setTargetPosition((int)COUNTS_PER_INCH*12);
                   Lift2.setTargetPosition((int)COUNTS_PER_INCH*12);
                   Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                   Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift.setPower(1);
                    Lift2.setPower(1);}
                else if(!armtoggle2 &&  (Math.abs(Lift.getCurrentPosition()/COUNTS_PER_INCH) > 0) && (Math.abs(Lift2.getCurrentPosition()/COUNTS_PER_INCH)>0)){
                    Lift.setTargetPosition((int)COUNTS_PER_INCH*0);
                Lift2.setTargetPosition((int)COUNTS_PER_INCH*0);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(-1);
                Lift2.setPower(-1);}

                else{  Lift.setPower(0);
                    Lift2.setPower(0);}

                if ((Math.abs(Lift.getCurrentPosition()/COUNTS_PER_INCH) > 6) && (Math.abs(Lift2.getCurrentPosition()/COUNTS_PER_INCH)>6)){
                    armservo.setPosition(.2);
                }
                else if (gamepad1.a && (Math.abs(Lift.getCurrentPosition()/COUNTS_PER_INCH) > 6) && (Math.abs(Lift2.getCurrentPosition()/COUNTS_PER_INCH)>6)){

                    armservo.setPosition(.4);
                }

                else {armservo.setPosition(0);}




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
                I.setPower(IntakePower);
                LR.setPower(LRPower);
                LF.setPower(LFPower);
                RR.setPower(RRPower);
                RF.setPower(RFPower);
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
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