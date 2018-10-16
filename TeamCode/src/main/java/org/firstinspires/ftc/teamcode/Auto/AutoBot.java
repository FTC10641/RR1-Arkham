package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.BB13HW;
import org.firstinspires.ftc.teamcode.Hardware.BB13Sensors;

import java.util.Locale;

/**
 * Created by Brady on 1/3/2018.
 */

@Autonomous(name = "AutoBot")


public class AutoBot extends OpMode {

    BB13HW robot = new BB13HW();
    BB13Sensors sensors = new BB13Sensors();

    enum State {
        Start,DriveForward,BackUp,TurnAround, Stop, Sample, DriveBack}

    State state;
    ElapsedTime time;

    @Override
    public void init() {
        robot.init(hardwareMap);
        sensors.initSensors(hardwareMap);
        state = State.Start;
        time = new ElapsedTime();

    }

    @Override
    public void loop() {
        double CurrentTime = time.time();
        telemetry.addData("time", CurrentTime);
        double gyroangle;
        sensors.angles = sensors.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroangle = Double.parseDouble(formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));
        telemetry.addData("Heading", formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));

        switch (state) {

            case Start:
                robot.Kill();
                if (robot.IsBusy()) {
                     state = State.DriveForward;
                    time.reset();
                }
                break;

            case DriveForward:
                robot.Forward(1,48);
                if(robot.DriveDone(48)){
                    state = State.BackUp;
                    time.reset();
                    robot.Kill();
                }
                break;

            case BackUp:
                robot.Reverse(1,6);
                if (robot.DriveDone(6)){
                    state = State.TurnAround;
                    time.reset();
                    robot.Kill();
                }
                break;

            case TurnAround:
                robot.TurnAbsoulte(-90,gyroangle);
                if (gyroangle>=-92&&gyroangle<=-88&& CurrentTime>=1.5){
                    state = State.DriveBack;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DriveBack:
                robot.Forward(1,42);
                if (robot.DriveDone(42)){
                    time.reset();
                    robot.Kill();
                    state = State.Stop;
                }
                break;

            case Stop:
                time.reset();
                robot.Kill();
                break;

        }
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    }
