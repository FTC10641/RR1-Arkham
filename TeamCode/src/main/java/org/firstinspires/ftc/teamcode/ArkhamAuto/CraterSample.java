package org.firstinspires.ftc.teamcode.ArkhamAuto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.SubSystems.ArkhamHW;
import org.firstinspires.ftc.teamcode.SubSystems.ArkhamSensors;
import org.firstinspires.ftc.teamcode.SubSystems.ArkhamVision;

import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.SubSystems.ArkhamVision.LABEL_GOLD_MINERAL;


@Autonomous(name = "CraterSample")


public class CraterSample extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();
    ArkhamVision vision = new ArkhamVision();

    enum State {
        Detach,DriveOff,LiftDown,Left,Center,Right, CenterKnockOffGold,CenterReverse, CenterTurn1,CenterForward1,CenterTurn2, CenterForward2, Sample,
        Backup, LeftTurn1, LeftTurn2, LeftForward1, Delay, Reverse, LeftForward2, LeftKnockOffGold, RightKnockOffGold, RightReverse1, RightTurn1, RigtForward1, RightTurn2, RightForward2, Stop
    }

    State state;
    ElapsedTime time;
    boolean right = false;
    boolean left = false;
    boolean center = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        sensors.initSensors(hardwareMap);
        vision.initVision(hardwareMap);
        state = State.Detach;
        time = new ElapsedTime();
        robot.LiftMotor.setPower(.09);
        robot.LiftMotor2.setPower(.09);
    }

    @Override
    public void loop() {
        double CurrentTime = time.time();
        telemetry.addData("time", CurrentTime);
        double gyroangle;
        sensors.angles = sensors.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroangle = Double.parseDouble(formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));
        telemetry.addData("Heading", formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));
        telemetry.addData("Current State",state.toString());
        telemetry.update();


        switch (state) {


            case Detach:
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = State.DriveOff;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes down and lands**/

            case DriveOff:
                robot.Forward(1, 2);
                if (robot.DriveDone(2)) {
                    state = State.LiftDown;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes forward to bring the latch completely off the lander **/

            case LiftDown:
                robot.LiftMotor.setPower(1);
                robot.LiftMotor2.setPower(-1);
                if (!robot.BottomSwitch.getState() == true){
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = State.Sample;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** the lift goes back down**/


            case Sample:if (vision.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = vision.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            }
                            else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            }
                            else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if ((goldMineralX != -1 && silverMineral1X !=-1)|| (goldMineralX != -1 && silverMineral2X !=-1) || (silverMineral1X != -1 && silverMineral2X !=-1)) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                state = State.Left;
                                left = true;
                                time.reset();
                                robot.Kill();
                            }
                            else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                state = State.Right;
                                time.reset();
                                right = true;
                                robot.Kill();
                            }
                            else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                state = State.Center;
                                center = true;
                                time.reset();
                                robot.Kill();
                            }
                        }
                    }
                }
            }
                break;

            case Left:
                robot.TurnAbsolute(25,gyroangle);
                if (gyroangle>=23 && gyroangle<=27){
                    state = State.LeftKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case Center:
                robot.TurnAbsolute(0,gyroangle);
                if (gyroangle>=-2 && gyroangle<=2) {
                    state = State.CenterKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case Right:
                robot.TurnAbsolute(-25,gyroangle);
                if (gyroangle>=-27 && gyroangle<=-23){
                    state = State.RightKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftKnockOffGold:
                robot.Forward(1, 35);
                robot.Intake.setPower(-1);
                if (robot.DriveDone(35)) {
                    robot.Intake.setPower(0);
                    state = State.LeftTurn1 ;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** This State makes the robot drive forward to knock off the gold while the intake is going outward so the gold doesn't get in the way **/

            case LeftTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92) {
                    state = CraterSample.State.LeftForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftForward1:
                robot.Forward(1, 25);
                if (robot.DriveDone(25)) {
                    state = CraterSample.State.LeftTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn2:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137) {
                    state = CraterSample.State.LeftForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftForward2:
                robot.Forward(1, 65);
                if (robot.DriveDone(65)) {
                    state = CraterSample.State.Delay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterKnockOffGold:
                robot.Forward(1, 35);
                robot.Intake.setPower(-1);
                if (robot.DriveDone(35)) {
                    robot.Intake.setPower(0);
                    state = State.CenterReverse;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterReverse:
                robot.Reverse(1, 14);
                if (robot.DriveDone(14)) {
                    state = State.CenterTurn1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92) {
                    state = State.CenterForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterForward1:
                robot.Forward(1, 60);
                if (robot.DriveDone(60)) {
                    state = State.CenterTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn2:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137) {
                    state = State.CenterForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterForward2:
                robot.Forward(1, 48);
                if (robot.DriveDone(48)) {
                    state = State.Delay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightKnockOffGold:
                robot.Forward(1, 35);
                robot.Intake.setPower(-1);
                if (robot.DriveDone(35)) {
                    robot.Intake.setPower(0);
                    state = State.RightReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightReverse1:
                robot.Reverse(1, 14);
                if (robot.DriveDone(14)) {
                    state = State.RightTurn1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92) {
                    state = State.RigtForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RigtForward1:
                robot.Forward(1, 55);
                if (robot.DriveDone(55)) {
                    state = State.RightTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn2:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137) {
                    state = State.RightForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightForward2:
                robot.Forward(1, 65);
                if (robot.DriveDone(65)) {
                    state = CraterSample.State.Delay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case Delay:
                if (CurrentTime >= 2.0) {
                    state = CraterSample.State.Reverse;
                    time.reset();
                    robot.Kill();
                }
                break;

            case Reverse:
                robot.Reverse(1, 87);
                if (robot.DriveDone(87)) {
                    state = CraterSample.State.Stop;
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

