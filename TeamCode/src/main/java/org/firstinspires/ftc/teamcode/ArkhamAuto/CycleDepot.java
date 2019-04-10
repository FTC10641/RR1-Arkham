
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


@Autonomous(name = "CycleDepot")


public class CycleDepot extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();
    ArkhamVision vision = new ArkhamVision();

    enum State {
        CycleDepotDetach, CycleDepotDriveOff, CycleDepotLiftDown,
        CycleDepotSample,  CycleDepotLeftTurn1, CycleDepotLeftForward1,
        CycleDepotLeftTurn2, CycleDepotLeftForward2, CycleDepotLeftTurn3,
        CycleDepotLeftForward3, CycleDepotLeftReverse1, CycleDepotLeftTurn4,
        CycleDepotLeftReverse2, CycleDepotLeftTurn5, CycleDepotLeftReverse3,
        CycleDepotLeftForward4, CycleDepotCenterReverse1, CycleDepotCenterTurn4,
        CycleDepotCenterReverse2, CycleDepotCenterTurn5, CycleDepotCenterReverse3,
        CycleDepotCenterTurn1, CycleDepotCenterForward1, CycleDepotCenterTurn2,
        CycleDepotCenterForward2, CycleDepotCenterTurn3, CycleDepotCenterForward3,
        CycleDepotCenterTurn6, CycleDepotCenterTurn7, CycleDepotCenterReverse4,
        LeftCycleScore, CenterCycleScore, CycleDepotCenterTurn8,
        CycleDepotCenterForward4, CycleDepotCenterTurn9, CycleDepotCenterForward5,
        CycleDepotRightTurn1, CycleDepotRightForward1, CycleDepotRightTurn2,
        CycleDepotRightForward2, CycleDepotRightTurn3, CycleDepotRightForward3,
        CycleDepotRightReverse1, CycleDepotRightTurn4, CycleDepotRightReverse2,
        CycleDepotRightTurn5, CycleDepotRightTurn6, CycleDepotRightReverse3,
        CycleDepotRightTurn7, CycleDepotRightReverse4, RightCycleScore,
        CycleDepotRightTurn8, CycleDepotRightForward4, CycleDepotRightTurn9,
        CycleDepotRightForward5, Delay, CycleDepotLeftExtendo1, CycleDepotLeftRetracto1,
        CycleDepotLeftForward5, CycleDepotLeftTurn6, CycleDepotLeftForward6,
        CycleDepotLeftTurn7, CycleDepotLeftExtendo2, CycleDepotLeftRetracto2,
        CycleDepotLeftTurn8, CycleDepotLeftTurn9, LeftCycleScore1,
        LeftCycleScore2, CycleDepotCenterExtendo1, CycleDepotCenterRetracto1,
        CycleDepotCenterForward6, CycleDepotCenterExtendo2, CycleDepotCenterRetracto2,
        CenterCycleScore1, CycleDepotRightExtendo1, CycleDepotRightRetracto1, Stop
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
        state = State.CycleDepotDetach;
        ElapsedTime runtime = new ElapsedTime();
        robot.LiftMotor.setPower(.2);
        robot.LiftMotor2.setPower(.2);
        robot.ArmServo.setPosition(0.015);
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


            case CycleDepotDetach:
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = CycleDepot.State.CycleDepotDriveOff;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes down and lands**/

            case CycleDepotDriveOff:
                robot.Forward(1, 15);
                if (robot.DriveDone(15)) {
                    state = State.CycleDepotLiftDown;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes forward to bring the latch completely off the lander **/

            case CycleDepotLiftDown:
                robot.LiftMotor.setPower(1);
                robot.LiftMotor2.setPower(-1);
                if (!robot.BottomSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = State.CycleDepotSample;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotSample:
                if (CurrentTime >= 4) {
                    telemetry.addData("Gold Mineral Position", "Center");
                    state = State.CycleDepotCenterTurn1;
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
                                    state = State.CycleDepotLeftTurn1;
                                    left = true;
                                    time.reset();
                                    robot.Kill();
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    state = State.CycleDepotRightTurn1;
                                    time.reset();
                                    right = true;
                                    robot.Kill();
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    state = State.CycleDepotCenterTurn1;
                                    center = true;
                                    time.reset();
                                    robot.Kill();
                                }
                            }
                        }
                    }
                }
                break;


            case CycleDepotLeftTurn1:
                        robot.TurnAbsolute(45, gyroangle);
                        if (gyroangle >= 43 && gyroangle <= 47 && CurrentTime >= 1.0) {
                            state = State.CycleDepotLeftForward1;
                            time.reset();
                            robot.Kill();
                        }
                        break;

            case CycleDepotLeftForward1:
                        robot.Forward(1, 50);
                        if (robot.DriveDone(50)) {
                            state = State.CycleDepotLeftTurn2;
                            time.reset();
                            robot.Kill();
                        }
                        break;

            case CycleDepotLeftTurn2:
                        robot.TurnAbsolute(-45, gyroangle);
                        if (gyroangle >= -47 && gyroangle <= -43 && CurrentTime >= 1.0) {
                            state = State.CycleDepotLeftForward2;
                            time.reset();
                            robot.Kill();
                        }
                        break;

            case CycleDepotLeftForward2:
                        robot.Forward(1, 25);
                        if (robot.DriveDone(25)) {
                            state = State.CycleDepotLeftExtendo1;
                            time.reset();
                            robot.Kill();
                        }
                        break;

            case CycleDepotLeftExtendo1:
                robot.Extendo.setPower(1);
                if (!robot.OutSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower(1);
                    if (CurrentTime >= .25) {
                        robot.IntakeServo.setPosition(1);
                    }
                    if (CurrentTime >= 0.5) {
                        state = CycleDepot.State.CycleDepotLeftRetracto1;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleDepotLeftRetracto1:
                robot.Extendo.setPower(-1);
                robot.Intake.setPower (1);
                if (CurrentTime >= 0.25) {
                    robot.IntakeServo.setPosition(.1);
                }
                if (!robot.InSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower (0);
                    state = CycleDepot.State.CycleDepotLeftTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftTurn3:
                robot.TurnAbsolute(177, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= 175 && gyroangle <= 179 && CurrentTime >= 1) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotLeftForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftForward3:
                robot.Reverse(1, 25);
                robot.Intake.setPower(-1);
                if (robot.DriveDone(25)) {
                    robot.Intake.setPower(0);
                    state = State.CycleDepotLeftTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftTurn4:
                robot.TurnAbsolute(-90, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= -92 && gyroangle <= -88 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotLeftForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftForward4:
                robot.Reverse(1, 25);
                robot.Intake.setPower (-1);
                if (robot.DriveDone(25)) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotLeftTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftTurn5:
                robot.TurnAbsolute(0, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotLeftReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftReverse1:
                robot.Reverse(1, 25);
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                robot.Intake.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                }
                if (robot.DriveDone(25)) {
                    robot.Intake.setPower(0);
                    state = State.LeftCycleScore1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftCycleScore1:
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
                        state = State.CycleDepotLeftForward5;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleDepotLeftForward5:
                robot.Reverse(1, 25);
                if (robot.DriveDone(25)) {
                    state = State.CycleDepotLeftTurn6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftTurn6:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.CycleDepotLeftForward6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftForward6:
                robot.Reverse(1, 60);
                if (robot.DriveDone(60)) {
                    state = State.CycleDepotLeftTurn7;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftTurn7:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137 && CurrentTime >= 1.0) {
                    state = State.CycleDepotLeftExtendo2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftExtendo2:
                robot.Extendo.setPower(1);
                if (!robot.OutSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower(1);
                    if (CurrentTime >= .25) {
                        robot.IntakeServo.setPosition(1);
                    }
                    if (CurrentTime >= 0.5) {
                        state = CycleDepot.State.CycleDepotLeftRetracto2;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleDepotLeftRetracto2:
                robot.Extendo.setPower(-1);
                robot.Intake.setPower (1);
                if (CurrentTime >= 0.25) {
                    robot.IntakeServo.setPosition(.1);
                }
                if (!robot.InSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower (0);
                    state = CycleDepot.State.CycleDepotLeftTurn8;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftTurn8:
                robot.TurnAbsolute(90, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotLeftReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftReverse2:
                robot.Reverse(1, 60);
                robot.Intake.setPower (-1);
                if(robot.DriveDone(60)) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotLeftTurn9;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftTurn9:
                robot.TurnAbsolute(0, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotLeftReverse3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotLeftReverse3:
                robot.Reverse(1, 25);
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                robot.Intake.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                }
                if (robot.DriveDone(25)) {
                    robot.Intake.setPower(0);
                    state = State.LeftCycleScore2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftCycleScore2:
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
                        state = State.Stop;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            /**End of the Left version of the code**/

            case CycleDepotCenterTurn1:
                robot.TurnAbsolute(45, gyroangle);
                if (gyroangle >= 43 && gyroangle <= 47 && CurrentTime >= 1.0) {
                    state = State.CycleDepotCenterForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterForward1:
                robot.Forward(1, 50);
                if (robot.DriveDone(50)) {
                    state = State.CycleDepotCenterTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterTurn2:
                robot.TurnAbsolute(-45, gyroangle);
                if (gyroangle >= -47 && gyroangle <= -43 && CurrentTime >= 1.0) {
                    state = State.CycleDepotCenterForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterForward2:
                robot.Forward(1, 20);
                if (robot.DriveDone(20)) {
                    state = State.CycleDepotCenterExtendo1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterExtendo1:
                robot.Extendo.setPower(1);
                if (!robot.OutSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower(1);
                    if (CurrentTime >= .3) {
                        robot.IntakeServo.setPosition(1);
                    }
                    if (CurrentTime >= 1.5) {
                        state = CycleDepot.State.CycleDepotCenterRetracto1;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleDepotCenterRetracto1:
                robot.Extendo.setPower(-1);
                robot.Intake.setPower (1);
                if (CurrentTime >= 0.3) {
                    robot.IntakeServo.setPosition(.1);
                }
                if (!robot.InSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower (0);
                    state = CycleDepot.State.CycleDepotCenterTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterTurn3:
                robot.TurnAbsolute(-90, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= -92 && gyroangle <= -88 && CurrentTime >= 1) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotCenterForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterForward3:
                robot.Reverse(1, 20);
                robot.Intake.setPower(-1);
                if (robot.DriveDone(20)) {
                    robot.Intake.setPower(0);
                    state = State.CycleDepotCenterTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterTurn4:
                robot.TurnAbsolute(177, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= 175 && gyroangle <= 179 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotCenterForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterForward4:
                robot.Reverse(1, 20);
                robot.Intake.setPower (-1);
                if (robot.DriveDone(20)) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotCenterTurn6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterTurn5:
                robot.TurnAbsolute(90, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotCenterReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterReverse1:
                robot.Reverse(1, 10);
                robot.Intake.setPower (-1);
                if (robot.DriveDone(10)) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotCenterTurn6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterTurn6:
                robot.TurnAbsolute(0, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotCenterReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterReverse2:
                robot.Reverse(1, 25);
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                }
                if (robot.DriveDone(25)) {
                    state = CycleDepot.State.CenterCycleScore1;
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
                        state = State.CycleDepotCenterForward5;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleDepotCenterForward5:
                robot.Forward(1, 25);
                if (robot.DriveDone(25)) {
                    state = State.CycleDepotCenterTurn7;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterTurn7:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.CycleDepotCenterForward6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterForward6:
                robot.Forward(1, 50);
                if (robot.DriveDone(50)) {
                    state = State.CycleDepotCenterTurn8;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterTurn8:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137 && CurrentTime >= 1.0) {
                    state = State.CycleDepotCenterExtendo2;
                    time.reset();
                    robot.Kill();
                }
                break;


            case CycleDepotCenterExtendo2:
                robot.Extendo.setPower(1);
                if (!robot.OutSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower(1);
                    if (CurrentTime >= .25) {
                        robot.IntakeServo.setPosition(1);
                    }
                    if (CurrentTime >= 0.5) {
                        state = CycleDepot.State.CycleDepotCenterRetracto2;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleDepotCenterRetracto2:
                robot.Extendo.setPower(-1);
                robot.Intake.setPower (1);
                if (CurrentTime >= 0.25) {
                    robot.IntakeServo.setPosition(.1);
                }
                if (!robot.InSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower (0);
                    state = CycleDepot.State.CycleDepotCenterTurn9;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterTurn9:
                robot.TurnAbsolute(90, gyroangle);
                robot.Intake.setPower (-1);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    robot.Intake.setPower (0);
                    state = State.CycleDepotCenterExtendo2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotCenterReverse3:
                robot.Reverse(1, 50);
                robot.Intake.setPower (-1);
                if (robot.DriveDone(50)) {
                    robot.Intake.setPower (0);
                    state = State.CenterCycleScore;
                    time.reset();
                    robot.Kill();
                }
                break;

            /**End of the Center version of the code**/

            case CycleDepotRightTurn1:
                robot.TurnAbsolute(45, gyroangle);
                if (gyroangle >= 43 && gyroangle <= 47 && CurrentTime >= 1.0) {
                    state = State.CycleDepotRightForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightForward1:
                robot.Forward(1, 50);
                if (robot.DriveDone(50)) {
                    state = State.CycleDepotRightTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightTurn2:
                robot.TurnAbsolute(-45, gyroangle);
                if (gyroangle >= -47 && gyroangle <= -43 && CurrentTime >= 1.0) {
                    state = State.CycleDepotRightForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightForward2:
                robot.Forward(1, 25);
                if (robot.DriveDone(25)) {
                    state = State.CycleDepotRightExtendo1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightExtendo1:
                robot.Extendo.setPower(1);
                if (!robot.OutSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower(1);
                    if (CurrentTime >= .25) {
                        robot.IntakeServo.setPosition(1);
                    }
                    if (CurrentTime >= 0.5) {
                        state = CycleDepot.State.CycleDepotRightRetracto1;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleDepotRightRetracto1:
                robot.Extendo.setPower(-1);
                robot.Intake.setPower (1);
                if (CurrentTime >= 0.25) {
                    robot.IntakeServo.setPosition(.1);
                }
                if (!robot.InSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower (0);
                    state = CycleDepot.State.CycleDepotRightTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightTurn3:
                robot.TurnAbsolute(-90, gyroangle);
                if (gyroangle >= -92 && gyroangle <= -88 && CurrentTime >= 1) {
                    state = State.CycleDepotRightForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightForward3:
                robot.Forward(1, 25);
                if (robot.DriveDone(25)) {
                    state = State.CycleDepotRightTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightTurn4:
                robot.TurnAbsolute(177, gyroangle);
                if (gyroangle >= 175 && gyroangle <= 179 && CurrentTime >= 1) {
                    state = State.CycleDepotRightForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightForward4:
                robot.Forward(1, 25);
                if (robot.DriveDone(25)) {
                    state = State.CycleDepotRightTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightTurn5:
                robot.TurnAbsolute(177, gyroangle);
                if (gyroangle >= 175 && gyroangle <= 179 && CurrentTime >= 1) {
                    state = State.CycleDepotRightForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightReverse1:
                robot.Reverse(1, 18);
                robot.Intake.setPower(1);
                if (robot.DriveDone(18)) {
                    robot.Intake.setPower(0);
                    state = State.CycleDepotRightTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;


            case CycleDepotRightReverse2:
                robot.Reverse(1, 35);
                if (robot.DriveDone(35)) {
                    state = State.CycleDepotRightTurn6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightTurn6:
                robot.TurnAbsolute(140, gyroangle);
                robot.Intake.setPower (1);
                if (gyroangle >= 138 && gyroangle <= 142 && CurrentTime >= 1.0) {
                    state = State.CycleDepotRightReverse3;
                    robot.Intake.setPower (0);
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightReverse3:
                robot.Reverse(1, 10);
                if (robot.DriveDone(10)) {
                    state = State.CycleDepotRightTurn7;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightTurn7:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    state = State.CycleDepotRightReverse4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightReverse4:
                robot.Reverse(1, 10);
                if (robot.DriveDone(10)) {
                    state = State.RightCycleScore;
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
                        state = State.CycleDepotRightTurn8;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleDepotRightTurn8:
                robot.TurnAbsolute(-50, gyroangle);
                if (gyroangle >= -52 && gyroangle <= -48 && CurrentTime >= 1.0) {
                    state = State.CycleDepotRightForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightTurn9:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    state = State.CycleDepotRightForward5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleDepotRightForward5:
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
