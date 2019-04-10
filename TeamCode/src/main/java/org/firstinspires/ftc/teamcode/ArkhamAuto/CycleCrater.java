
package org.firstinspires.ftc.teamcode.ArkhamAuto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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


@Autonomous(name = "CycleCrater")


public class CycleCrater extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();
    ArkhamVision vision = new ArkhamVision();

    enum State {
        CycleCraterDetach, CycleCraterDriveOff, CycleCraterLiftDown,
        CycleCraterSample,  CycleCraterLeftTurn1, CycleCraterLeftForward1,
        CycleCraterLeftTurn2, CycleCraterLeftForward2, CycleCraterLeftTurn3,
        CycleCraterLeftForward3, CycleCraterLeftReverse1, CycleCraterLeftTurn4,
        CycleCraterLeftReverse2, CycleCraterLeftTurn5, LeftExtendo2,
        CycleCraterLeftReverse3, CycleCraterLeftForward4, CycleCraterCenterReverse1,
        CycleCraterCenterTurn4, CycleCraterCenterReverse2, CycleCraterCenterTurn5,
        CenterExtendo2, CycleCraterCenterReverse3, CycleCraterCenterTurn1,
        CycleCraterCenterForward1, CycleCraterCenterTurn2, CycleCraterCenterForward2,
        CycleCraterCenterTurn3, CycleCraterCenterForward3, CenterExtendo1,
        CenterRetracto1, CycleCraterCenterTurn6, CycleCraterCenterTurn7,
        CycleCraterCenterReverse4, CycleCraterCenterTurn8, CycleCraterCenterForward4,
        CycleCraterCenterTurn9, CycleCraterCenterForward5, LeftRetracto2,
        CenterRetracto2, CycleCraterRightTurn1, CycleCraterRightForward1,
        CycleCraterRightTurn2, CycleCraterRightForward2, CycleCraterRightTurn3,
        CycleCraterRightForward3, RightExtendo1, RightRetracto1, CycleCraterRightReverse1,
        CycleCraterRightTurn4, CycleCraterRightReverse2, CycleCraterRightTurn5,
        RightExtendo2, RightRetracto2, CycleCraterRightTurn6, CycleCraterRightReverse3,
        CycleCraterRightTurn7, CycleCraterRightReverse4, RightCycleScore,
        CycleCraterRightTurn8, CycleCraterRightForward4, CycleCraterRightTurn9,
        CycleCraterRightForward5, Delay, CycleCraterLeftExtendo1, CycleCraterLeftRetracto1,
        LeftCycleScore1, CenterCycleScore1, CycleCraterLeftForward5, CycleCraterLeftMarkerDrop,
        CycleCraterLeftMarkerDrop2, CycleCraterLeftTurn6, CycleCraterLeftReverse4,
        CycleCraterLeftTurn7, Intake1, Stop
    }

    State state;
    ElapsedTime time = new ElapsedTime();
    boolean right = false;
    boolean left = false;
    boolean center = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        sensors.initSensors(hardwareMap);
        vision.initVision(hardwareMap);
        state = State.LeftExtendo2;
        ElapsedTime runtime = new ElapsedTime();
        robot.LiftMotor.setPower(.2);
        robot.LiftMotor2.setPower(.2);
        robot.ArmServo.setPosition(0.015);
        robot.BackServo.setPosition(0.4);
        robot.IntakeServo.setPosition(0.1);
        robot.Extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


            case CycleCraterDetach:
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = CycleCrater.State.CycleCraterDriveOff;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes down and lands**/

            case CycleCraterDriveOff:
                robot.Forward(1, 2);
                if (robot.DriveDone(2)) {
                    state = State.CycleCraterLiftDown;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes forward to bring the latch completely off the lander **/

            case CycleCraterLiftDown:
                robot.LiftMotor.setPower(1);
                robot.LiftMotor2.setPower(-1);
                if (!robot.BottomSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = State.CycleCraterSample;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterSample:
                if (CurrentTime >= 4) {
                    telemetry.addData("Gold Mineral Position", "Center");
                    state = State.CycleCraterCenterTurn1;
                    center = true;
                    time.reset();
                    robot.Kill();
                }

                if (vision.tfod != null) {
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
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if ((goldMineralX != -1 && silverMineral1X != -1) || (goldMineralX != -1 && silverMineral2X != -1)
                                || (silverMineral1X != -1 && silverMineral2X != -1)) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    state = State.CycleCraterLeftForward1;
                                    left = true;
                                    time.reset();
                                    robot.Kill();
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    state = State.CycleCraterRightTurn1;
                                    time.reset();
                                    right = true;
                                    robot.Kill();
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    state = State.CycleCraterCenterTurn1;
                                    center = true;
                                    time.reset();
                                    robot.Kill();
                                }
                            }
                        }
                    }
                }
                break;


            /** This State makes the robot drive forward to knock off the gold while the intake is going inward to pick it up**/

            case CycleCraterLeftForward1:
                robot.Forward(1, 18);
                if (robot.DriveDone(18)) {
                    state = State.CycleCraterLeftTurn1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.CycleCraterLeftForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftForward2:
                robot.Forward(1, 25);
                if (robot.DriveDone(25)) {
                    state = State.CycleCraterLeftTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftTurn2:
                robot.TurnAbsolute(120, gyroangle);
                if (gyroangle >= 118 && gyroangle <= 122 && CurrentTime >= 1.0) {
                    state = State.CycleCraterLeftForward3;
                    time.reset();
                    robot.Kill();
                }
                    break;

            case CycleCraterLeftForward3:
                robot.Forward(1, 18);
                if (robot.DriveDone(18)) {
                    state = State.CycleCraterLeftTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftTurn3:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137 && CurrentTime >= 1) {
                    state = State.CycleCraterLeftForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftForward4:
                robot.Forward(1, 15);
                if (robot.DriveDone(15)) {
                    state = State.CycleCraterLeftExtendo1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftExtendo1:
                robot.Extendo(1, 30);
                if (robot.DriveDone(13)) {
                    state = State.CycleCraterLeftMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftMarkerDrop:
                robot.IntakeServo.setPosition(1);
                if (robot.IntakeServo.getPosition() >= 1) {
                    robot.Intake.setPower(-1);
                    state = State.CycleCraterLeftMarkerDrop2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftMarkerDrop2:
                robot.IntakeServo.setPosition(0.1);
                if (robot.IntakeServo.getPosition() >= 0.1) {
                    state = State.CycleCraterLeftRetracto1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftRetracto1:
                robot.Retracto(1, 30);
                if (robot.DriveDone(13)) {
                    state = State.CycleCraterLeftReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftReverse1:
                robot.Reverse(1, 18);
                robot.Intake.setPower(-1);
                if (robot.DriveDone(18)) {
                    robot.Intake.setPower(0);
                    state = State.CycleCraterLeftTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftTurn4:
                robot.TurnAbsolute(90, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleCraterLeftReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftReverse2:
                robot.Reverse(1, 25);
                robot.Intake.setPower (-1);
                if (robot.DriveDone(25)) {
                    robot.Intake.setPower (0);
                    state = State.CycleCraterLeftTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftTurn5:
                robot.TurnAbsolute(0, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.LeftExtendo2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftExtendo2:
                robot.Extendo(1, 30);
                if (robot.DriveDone(13)) {
                   state = State.Intake1;
                   time.reset();
                    robot.Kill();
                }
                break;

            case Intake1:
                robot.IntakeServo.setPosition(1);
                robot.Intake.setPower(1);
                    if (CurrentTime >= 1.5) {
                        robot.IntakeServo.setPosition(.1);
                        state = State.LeftRetracto2;
                        time.reset();
                        robot.Kill();
                    }
                    break;

            case LeftRetracto2:
                    robot.Retracto(1,30);
                    if (robot.DriveDone(13)){
                        robot.Intake.setPower(-1);
                        if (CurrentTime >= 0.5) {
                            state = State.Stop;
                            time.reset();
                            robot.Kill();
                        }
                    }
                    break;

            case CycleCraterLeftTurn6:
                robot.Intake.setPower(-1);
                robot.TurnAbsolute(-90, gyroangle);
                if (gyroangle >= -92 && gyroangle <= -88 && CurrentTime >= 1.0) {
                    state = State.CycleCraterLeftReverse3;
                    time.reset();
                    robot.Kill();
                }

            case CycleCraterLeftReverse3:
                robot.Intake.setPower(-1);
                robot.Reverse(1,20);
                if (robot.DriveDone(20)){
                    state = State.CycleCraterLeftReverse4;
                    time.reset();
                    robot.Kill();
                }

            case CycleCraterLeftTurn7:
                robot.Intake.setPower(-1);
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0){
                    state = State.CycleCraterLeftReverse4;
                    time.reset();
                    robot.Kill();
                }

            case CycleCraterLeftReverse4:
                robot.Reverse(1, 15);
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                }
                if (robot.DriveDone(15)) {
                    state = State.LeftCycleScore1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftCycleScore1:
                robot.ArmServo.setPosition(0.12);
                if (CurrentTime >= 1) {
                    robot.ArmServo.setPosition(0.015);
                    if (robot.ArmServo.getPosition() == 0.015){
                        robot.LiftMotor.setPower(1);
                        robot.LiftMotor2.setPower(-1);
                    }
                    if (!robot.BottomSwitch.getState() == true) {
                        robot.LiftMotor.setPower(0);
                        robot.LiftMotor2.setPower(0);
                        state = State.CycleCraterLeftForward4;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleCraterLeftForward5:
                robot.Forward(1, 40);
                if (robot.DriveDone(40)) {
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;

            /**End of the Left version of the code**/

            case CycleCraterCenterTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.CycleCraterCenterForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterForward1:
                robot.Forward(1, 30);
                if (robot.DriveDone(30)) {
                    state = State.CycleCraterCenterTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterTurn2:
                robot.TurnAbsolute(120, gyroangle);
                if (gyroangle >= 118 && gyroangle <= 122 && CurrentTime >= 1.0) {
                    state = State.CycleCraterCenterForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterForward2:
                robot.Forward(1, 18);
                if (robot.DriveDone(18)) {
                    state = State.CycleCraterCenterTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterTurn3:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137 && CurrentTime >= 1) {
                    state = State.CycleCraterCenterForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterForward3:
                robot.Forward(1, 18);
                if (robot.DriveDone(18)) {
                    state = State.CenterExtendo1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterExtendo1:
                robot.Extendo.setPower(1);
                if (!robot.OutSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower(1);
                    if (CurrentTime >= 0.5) {
                        state = State.CenterRetracto1;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CenterRetracto1:
                robot.Extendo.setPower(-1);
                robot.Intake.setPower (1);
                if (!robot.InSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower (0);
                    state = State.CycleCraterCenterReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterReverse1:
                robot.Reverse(1, 18);
                if (robot.DriveDone(18)) {
                    state = State.CycleCraterCenterTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterTurn4:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.CycleCraterCenterReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterReverse2:
                robot.Reverse(1, 30);
                if (robot.DriveDone(30)) {
                    state = State.CycleCraterCenterTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterTurn5:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    state = State.CenterExtendo2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterExtendo2:
                robot.Extendo.setPower(1);
                if (!robot.OutSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower(1);
                    if (CurrentTime >= 0.5) {
                        state = State.CenterRetracto2;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CenterRetracto2:
                robot.Extendo.setPower(-1);
                robot.Intake.setPower (1);
                if (!robot.InSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower (0);
                    state = State.CycleCraterCenterTurn6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterTurn6:
                robot.TurnAbsolute(-25, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= -27 && gyroangle <= -23 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleCraterCenterReverse3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterReverse3:
                robot.Reverse(1, 7);
                robot.Intake.setPower (-1);
                if (robot.DriveDone(7)) {
                    robot.Intake.setPower (0);
                    state = State.CycleCraterCenterTurn7;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterTurn7:
                robot.TurnAbsolute(0, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleCraterCenterReverse4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterReverse4:
                robot.Reverse(1, 7);
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                }
                if (robot.DriveDone(7)) {
                    state = CycleCrater.State.CenterCycleScore1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterCycleScore1:
                robot.ArmServo.setPosition(0.12);
                if (CurrentTime >= 2) {
                    robot.ArmServo.setPosition(0.015);
                    if (robot.ArmServo.getPosition() == 0.015){
                        robot.LiftMotor.setPower(1);
                        robot.LiftMotor2.setPower(-1);
                    }
                    if (!robot.BottomSwitch.getState() == true) {
                        robot.LiftMotor.setPower(0);
                        robot.LiftMotor2.setPower(0);
                        state = State.CycleCraterCenterTurn8;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleCraterCenterTurn8:
                robot.TurnAbsolute(-25, gyroangle);
                if (gyroangle >= -27 && gyroangle <= -23 && CurrentTime >= 1.0) {
                    state = State.CycleCraterCenterForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterForward4:
                robot.Reverse(1, 7);
                if (robot.DriveDone(7)) {
                    state = State.CycleCraterCenterTurn9;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterTurn9:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    state = State.CycleCraterCenterForward5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterCenterForward5:
                robot.Reverse(1, 20);
                if (robot.DriveDone(20)) {
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;

            /**End of the Center version of the code**/

            case CycleCraterRightTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.CycleCraterRightForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightForward1:
                robot.Forward(1, 35);
                if (robot.DriveDone(35)) {
                    state = State.CycleCraterRightTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightTurn2:
                robot.TurnAbsolute(120, gyroangle);
                if (gyroangle >= 118 && gyroangle <= 122 && CurrentTime >= 1.0) {
                    state = State.CycleCraterRightForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightForward2:
                robot.Forward(1, 18);
                if (robot.DriveDone(18)) {
                    state = State.CycleCraterRightTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightTurn3:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137 && CurrentTime >= 1) {
                    state = State.CycleCraterRightForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightForward3:
                robot.Forward(1, 18);
                if (robot.DriveDone(18)) {
                    state = State.RightExtendo1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightExtendo1:
                robot.Extendo.setPower(1);
                if (!robot.OutSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower(1);
                    if (CurrentTime >= 0.5) {
                        state = State.RightRetracto1;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case RightRetracto1:
                robot.Extendo.setPower(-1);
                robot.Intake.setPower (1);
                if (!robot.InSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower (0);
                    state = State.CycleCraterRightReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightReverse1:
                robot.Reverse(1, 18);
                robot.Intake.setPower(1);
                if (robot.DriveDone(18)) {
                    robot.Intake.setPower(0);
                    state = State.CycleCraterRightTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightTurn4:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.CycleCraterRightReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightReverse2:
                robot.Reverse(1, 35);
                if (robot.DriveDone(35)) {
                    state = State.CycleCraterRightTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightTurn5:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    state = State.RightExtendo2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightExtendo2:
                robot.Extendo.setPower(1);
                if (!robot.OutSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower(1);
                    if (CurrentTime >= 0.5) {
                        state = State.RightRetracto2;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case RightRetracto2:
                robot.Extendo.setPower(-1);
                robot.Intake.setPower (1);
                if (!robot.InSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower (0);
                    state = State.CycleCraterRightTurn6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightTurn6:
                robot.TurnAbsolute(140, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= 138 && gyroangle <= 142 && CurrentTime >= 1.0) {
                    state = State.CycleCraterRightReverse3;
                    robot.Intake.setPower (0);
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightReverse3:
                robot.Reverse(1, 10);
                robot.Intake.setPower (-1);
                if (robot.DriveDone(10)) {
                    robot.Intake.setPower (0);
                    state = State.CycleCraterRightTurn7;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightTurn7:
                robot.TurnAbsolute(0, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleCraterRightReverse4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightReverse4:
                robot.Reverse(1, 10);
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                }
                if (robot.DriveDone(10)) {
                    state = CycleCrater.State.CenterCycleScore1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightCycleScore:
                robot.ArmServo.setPosition(0.12);
                if (CurrentTime >= 2) {
                    robot.ArmServo.setPosition(0.015);
                    if (robot.ArmServo.getPosition() == 0.015){
                        robot.LiftMotor.setPower(1);
                        robot.LiftMotor2.setPower(-1);
                    }
                    if (!robot.BottomSwitch.getState() == true) {
                        robot.LiftMotor.setPower(0);
                        robot.LiftMotor2.setPower(0);
                        state = State.CycleCraterRightTurn8;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleCraterRightTurn8:
                robot.TurnAbsolute(-50, gyroangle);
                if (gyroangle >= -52 && gyroangle <= -48 && CurrentTime >= 1.0) {
                    state = State.CycleCraterRightForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightForward4:
                robot.Reverse(1, 10);
                if (robot.DriveDone(10)) {
                    state = State.CycleCraterRightTurn9;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightTurn9:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    state = State.CycleCraterRightForward5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterRightForward5:
                robot.Reverse(1, 20);
                if (robot.DriveDone(20)) {
                    state = State.Delay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case Delay:
                if (CurrentTime >= .5) {
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }

            /**End of the Right version of the code**/

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
