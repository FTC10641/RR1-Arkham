package org.firstinspires.ftc.teamcode.ArkhamAuto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.SubSystems.ArkhamHW;
import org.firstinspires.ftc.teamcode.SubSystems.ArkhamSensors;

import java.util.Locale;

@Autonomous(name = "ArkhamAutoTestCenter")

public class ArkhamAutoTestCenter extends OpMode {


    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();

    enum State {
        CenterTurn2, CenterDriveForward2, MarkerDrop, Delay, CenterTurn3, CenterDriveForward3, CenterDriveReverse2, CenterDriveReverse1, Stop
    }

    State state;
    ElapsedTime time;

    @Override
    public void init() {
        robot.init(hardwareMap);
        sensors.initSensors(hardwareMap);
        state = State.CenterDriveReverse1;
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
        telemetry.update();

        switch (state) {


            case CenterDriveReverse1:
                robot.Reverse(1, 12);
                if (robot.DriveDone(12)) {
                    state = ArkhamAutoTestCenter.State.CenterTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn2:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92) {
                    state = ArkhamAutoTestCenter.State.CenterDriveForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterDriveForward2:
                robot.Forward(1, 45);
                if (robot.DriveDone(45)) {
                    state = ArkhamAutoTestCenter.State.CenterTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn3:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137) {
                    state = ArkhamAutoTestCenter.State.CenterDriveForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterDriveForward3:
                robot.Forward(1, 48);
                if (robot.DriveDone(48)) {
                    state = State.Delay;
                    time.reset();
                    robot.Kill();
                }
                break;

          /*  case MarkerDrop:
                robot.LeftSorterServo.setPosition(0.5);
                robot.RightSorterServo.setPosition(0.5);
                if(robot.LeftSorterServo.getPosition()>=0.5 && robot.RightSorterServo.getPosition()>=0.5){
                    state = ArkhamAutoTestCenter.State.CenterDriveReverse2;
                    time.reset();
                    robot.Kill();}
                break;*/

            case Delay:
                if (CurrentTime >= 3.0) {
                    state = State.CenterDriveReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;


            case CenterDriveReverse2:
                robot.Reverse(1, 70);
                if (robot.DriveDone(70)) {
                    state = ArkhamAutoTestCenter.State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case Stop:
                time.reset();
                robot.Kill();
                break;


        }
    }

    String formatAngle (AngleUnit angleUnit,double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees ( double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

