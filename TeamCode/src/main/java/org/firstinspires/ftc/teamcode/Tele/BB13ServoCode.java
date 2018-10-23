
package org.firstinspires.ftc.teamcode.Tele;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//Disabled
@TeleOp(name="owo", group="Linear Opmode")

public class BB13ServoCode extends owo
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo servo = null;
    private Servo servo2 = null;
    private boolean toggle = true;
    private boolean toggle2 = false;

    @Override
    public void runOpMode ()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        servo = hardwareMap.servo.get("ls");
        servo2 = hardwareMap.servo.get("rs");




        // Wait for the game to start (driver presses PLAY)
        servo.setPosition(1);
        servo2.setPosition(0);


        waitForStart();
        if (opModeIsActive()) {


            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                // if the digital channel returns true it's HIGH and the button is unpressed.

                if (toggle && gamepad1.left_bumper) {  // Only execute once per Button push
                    toggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                    if (toggle2) {
                        toggle2 = false;
                        servo.setPosition(0.5);
                        servo2.setPosition(0.5);
                    } else {
                        toggle2 = true;
                        servo.setPosition(1);
                        servo2.setPosition(0);
                    }
                } else if(gamepad1.left_bumper == false) {
                    toggle = true; // Button has been released, so this allows a re-press to activate the code above.
                }


                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Servo Position", servo.getPosition());
                telemetry.update();
            }
        }
    }
}