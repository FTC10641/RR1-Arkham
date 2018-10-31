
package org.firstinspires.ftc.teamcode.ArkhamAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ArkhamTele.ArkhamHW;
import org.firstinspires.ftc.teamcode.ArkhamTele.ArkhamSensors;
import org.firstinspires.ftc.teamcode.Hardware.BB13HW;
import org.firstinspires.ftc.teamcode.Hardware.BB13Sensors;

import java.util.Locale;

/**
 * Created by Brady on 1/3/2018.
 */

@Autonomous(name = "ArkhamAutoBot")


public class ArkhamAutoTest extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();

    enum State {
        Start,DriveForward,BackUp,TurnAround, Stop, Sample, DriveBack, TurnLeft, DriveForward2}

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
                robot.Forward(1,-48);
                if(robot.DriveDone(48)){
                    robot.Kill();
                    state = State.BackUp;
                    time.reset();

                }
                break;

            case BackUp:
                robot.Reverse(1,-6);
                if (robot.DriveDone(6)){
                    robot.Kill();
                    state = State.TurnAround;
                    time.reset();

                }
                break;

            case TurnAround:
                robot.TurnAbsolute(-90,gyroangle);
                if (gyroangle>=-92&&gyroangle<=-88&& CurrentTime>=1.5){
                    robot.Kill();
                    state = State.DriveBack;
                    time.reset();

                }
                break;

            case DriveBack:
                robot.Forward(1,-42);
                if (robot.DriveDone(42)){
                    robot.Kill();
                    state = State.TurnLeft;
                    time.reset();

                }
                break;

            case TurnLeft:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -1 && gyroangle <= 1) {
                    robot.Kill();
                    state = State.DriveForward2;
                    time.reset();

                }
                break;

            case DriveForward2:
                robot.Forward(1,-48);
                if(robot.DriveDone(76)){
                    robot.Kill();
                    state = State.Stop;
                    time.reset();

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
