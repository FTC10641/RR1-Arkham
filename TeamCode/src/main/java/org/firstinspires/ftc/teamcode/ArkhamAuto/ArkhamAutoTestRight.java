package org.firstinspires.ftc.teamcode.ArkhamAuto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.SubSystems.ArkhamHW;
import org.firstinspires.ftc.teamcode.SubSystems.ArkhamSensors;

import java.util.Locale;

@Autonomous(name = "ArkhamAutoTestRight")

public class ArkhamAutoTestRight extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();

    enum State {
        RightDriveReverse1, RightTurn1, RightDriveForward1, RightTurn2, RightDriveForward2, MarkerDrop, RightDriveReverse2, Stop
    }

    State state;
    ElapsedTime time;

    @Override
    public void init() {
        robot.init(hardwareMap);
        sensors.initSensors(hardwareMap);
        state = State.RightDriveReverse1;
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

            case RightDriveReverse1:
                robot.Reverse(1, 12);
                if (robot.DriveDone(12)) {
                    state = ArkhamAutoTestRight.State.RightTurn1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92) {
                    state = ArkhamAutoTestRight.State.RightDriveForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightDriveForward1:
                robot.Forward(1, 60);
                if (robot.DriveDone(60)) {
                    state = ArkhamAutoTestRight.State.RightTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn2:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137) {
                    state = ArkhamAutoTestRight.State.RightDriveForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightDriveForward2:
                robot.Forward(1, 55);
                if (robot.DriveDone(55)) {
                    state = State.RightDriveReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case MarkerDrop:
                robot.LeftSorterServo.setPosition(0.59);
                robot.RightSorterServo.setPosition(0.4);
                if(robot.LeftSorterServo.getPosition()>=0.59 && robot.RightSorterServo.getPosition()>=0.4){
                state = ArkhamAutoTestRight.State.RightDriveReverse2;
                time.reset();
                robot.Kill();}
                break;


            case RightDriveReverse2:
                robot.Reverse(1, 75);
                if (robot.DriveDone(75)) {
                    state = ArkhamAutoTestRight.State.Stop;
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