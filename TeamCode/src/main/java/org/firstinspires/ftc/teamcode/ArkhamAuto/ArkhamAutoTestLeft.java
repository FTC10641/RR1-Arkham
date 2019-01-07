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

@Autonomous(name = "ArkhamAutoTestLeft")

public class ArkhamAutoTestLeft extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();

    enum State {
        LeftTurn1, LeftDriveForward1, LeftTurn2, LeftDriveForward2, MarkerDrop, LeftDriveReverse, Stop
    }

    State state;
    ElapsedTime time;

    @Override
    public void init() {
        robot.init(hardwareMap);
        sensors.initSensors(hardwareMap);
        state = State.LeftTurn1;
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

            case LeftTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92) {
                    state = ArkhamAutoTestLeft.State.LeftDriveForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftDriveForward1:
                robot.Forward(1, 12);
                if (robot.DriveDone(12)) {
                    state = ArkhamAutoTestLeft.State.LeftTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn2:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137) {
                    state = ArkhamAutoTestLeft.State.LeftDriveForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftDriveForward2:
                robot.Forward(1, 65);
                if (robot.DriveDone(65)) {
                    state = ArkhamAutoTestLeft.State.MarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case MarkerDrop:
                robot.LeftSorterServo.setPosition(0.5);
                robot.RightSorterServo.setPosition(0.5);
                if(robot.LeftSorterServo.getPosition()>=0.5 && robot.RightSorterServo.getPosition()>=0.5){
                state = ArkhamAutoTestLeft.State.LeftDriveReverse;
                time.reset();
                robot.Kill();}
                break;


            case LeftDriveReverse:
                robot.Reverse(1, 70);
                if (robot.DriveDone(70)) {
                    state = ArkhamAutoTestLeft.State.Stop;
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
