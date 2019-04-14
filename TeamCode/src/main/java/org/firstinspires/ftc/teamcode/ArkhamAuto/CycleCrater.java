
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
        LeftTurn1, LeftForward1, LeftTurn2, LeftForward2,
        LeftTurn3, LeftForward3, LeftReverse1, LeftTurn4,
        LeftReverse2, LeftTurn5, LeftExtendo2,
        CenterReverse1, CenterTurn4, CenterReverse2,
        CenterTurn5, CenterExtendo2, CenterReverse3,
        CenterTurn1, CenterForward1, CenterTurn2,
        CenterForward2, CenterTurn3, CenterForward3,
        CenterExtendo1, CenterRetracto1, CenterTurn6,
        CenterTurn7, CenterReverse4, CenterTurn8,
        CenterTurn9, CenterForward5, LeftRetracto2,
        CenterRetracto2, RightTurn1, RightForward1,
        RightTurn2, RightForward2, RightTurn3, RightForward3,
        RightExtendo1, RightRetracto1, RightReverse1,
        RightTurn4, RightReverse2, RightTurn5, RightExtendo2,
        RightRetracto2, RightTurn6, RightReverse3,
        RightReverse4, LeftExtendo1, LeftRetracto1,
        LeftCycleScore1, CenterCycleScore1, LeftMarkerDrop,
        LeftMarkerDrop2, LeftShake1, LeftShake2,
        CenterForward6, CenterMarkerDrop, CenterMarkerDrop2,
        LeftIntake1, CenterIntake1, CenterIntake2,
        CenterShake2, CenterShake1, LeftIntake2,
        LeftReverse4, CraterSample, CraterLiftDown,
        CraterDriveOff, CraterDetach,  LeftForward4,
        RightMarkerDrop2, RightMarkerDrop, RightIntake2,
        RightIntake1, RightCycleScore1, RightShake1,
        RightShake2, LeftReverse3, LeftTurn6, LeftReverse5, LeftTurn7, LeftTurn8, LeftLiftUp1, LeftTurn9, LeftTurn10, CenterTurn10, RightTurn7, Stop
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
        state = State.CraterDetach;
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
        telemetry.addData("Current State", state.toString());
        telemetry.update();


        switch (state) {

            case CraterDetach:
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = State.CraterDriveOff;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes down and lands**/

            case CraterDriveOff:
                robot.ArmServo.setPosition(0.015);
                robot.Forward(1, 2);
                robot.IntakeServo.setPosition(0.5);
                if (robot.DriveDone(2)) {
                    state = State.CraterLiftDown;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes forward to bring the latch completely off the lander **/

            case CraterLiftDown:
                robot.ArmServo.setPosition(0.015);
                robot.LiftMotor.setPower(1);
                robot.LiftMotor2.setPower(-1);
                if (!robot.BottomSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = State.CraterSample;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CraterSample:
                if (CurrentTime >= 4) {
                    telemetry.addData("Gold Mineral Position", "Center");
                    state = State.CenterForward1;
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
                            if ((goldMineralX != -1 && silverMineral1X != -1) || (goldMineralX != -1 && silverMineral2X != -1) || (silverMineral1X != -1 && silverMineral2X != -1)) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    state = State.LeftForward1;
                                    left = true;
                                    time.reset();
                                    robot.Kill();
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    state = State.RightForward1;
                                    time.reset();
                                    right = true;
                                    robot.Kill();
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    state = State.CenterForward1;
                                    center = true;
                                    time.reset();
                                    robot.Kill();
                                }
                            }
                        }
                    }
                }
                break;

            case LeftForward1:
                robot.Forward(1, 20);
                robot.IntakeServo.setPosition(0.1);
                if (robot.DriveDone(20)) {
                    state = State.LeftTurn1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.LeftForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftForward2:
                robot.Forward(1, 52);
                if (robot.DriveDone(52)) {
                    state = State.LeftTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn2:
                robot.TurnAbsolute(120, gyroangle);
                if (gyroangle >= 118 && gyroangle <= 122) {
                    state = State.LeftForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftForward3:
                robot.Forward(1, 13);
                if (robot.DriveDone(13)) {
                    state = State.LeftTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn3:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137 && CurrentTime >= 1) {
                    state = State.LeftExtendo1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftExtendo1:
                robot.Extendo(1, 30);
                if (robot.DriveDone(13)) {
                    state = State.LeftMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftMarkerDrop:
                robot.IntakeServo.setPosition(1);
                if (CurrentTime  >= 0.5) {
                    time.reset();
                    state = State.LeftMarkerDrop2;
                    robot.Kill();
                }
                break;

            case LeftMarkerDrop2:
                robot.Intake.setPower(.5);
                if(CurrentTime >= 1.0) {
                    state = State.LeftRetracto1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftRetracto1:
                robot.IntakeServo.setPosition(0.1);
                robot.Retracto(1, 30);
                if (robot.DriveDone(13)) {
                    state = State.LeftTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn4:
                robot.Intake.setPower(0);
                robot.TurnAbsolute(120, gyroangle);
                if (gyroangle >= 118 && gyroangle <= 122 && CurrentTime >= 1.0) {
                    robot.Intake.setPower(0);
                    state = State.LeftReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftReverse1:
                robot.Reverse(1, 13);
                if (robot.DriveDone(13)) {
                    robot.Intake.setPower(0);
                    state = State.LeftTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn5:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    robot.Intake.setPower(0);
                    state = State.LeftReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftReverse2:
                robot.Reverse(1, 46);
                if (robot.DriveDone(46)) {
                    robot.Intake.setPower(0);
                    state = State.LeftTurn6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn6:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    robot.Intake.setPower(0);
                    state = State.LeftReverse3;
                    time.reset();
                    robot.Kill();
                }
                break;


            case LeftReverse3:
                robot.Reverse(1, 19);
                if (robot.DriveDone(19)) {
                    robot.Intake.setPower(0);
                    state = State.LeftTurn7;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn7:
                robot.TurnAbsolute(15, gyroangle);
                if (gyroangle >= 13 && gyroangle <= 17 && CurrentTime >= 1.0) {
                    robot.Intake.setPower(0);
                    state = State.LeftExtendo2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftExtendo2:
                robot.IntakeServo.setPosition(1);
                robot.Extendo(1, 30);
                if (robot.DriveDone(13)) {
                    state = State.LeftIntake1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftIntake1:
                robot.Intake.setPower(-1);
                if (CurrentTime >= 2) {
                    robot.IntakeServo.setPosition(.1);
                    state = State.LeftRetracto2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftRetracto2:
                robot.Retracto(1, 30);
                if (robot.DriveDone(10)) {
                    robot.Intake.setPower(-1);
                    if (CurrentTime >= 0.5) {
                        state = State.LeftReverse4;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case LeftReverse4:
                robot.Reverse(1, 1.5);
                robot.Intake.setPower(1);
                if (robot.DriveDone(1.5)) {
                    robot.Intake.setPower(0);
                    state = State.LeftTurn8;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn8:
                robot.TurnAbsolute(-15, gyroangle);
                robot.Intake.setPower(1);
                if (gyroangle >= -15 && gyroangle <= -8 && CurrentTime >= 1.0) {
                    robot.Intake.setPower(0);
                    state = State.LeftLiftUp1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftLiftUp1:
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = State.LeftCycleScore1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftCycleScore1:
                robot.ArmServo.setPosition(1);
                if (robot.ArmServo.getPosition() >= 0.12) {
                    state = State.LeftShake1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftShake1:
                robot.Reverse(1, 1);
                if (robot.DriveDone(1)) {
                    state = State.LeftTurn9;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn9:
                robot.TurnAbsolute(10, gyroangle);
                if (gyroangle >= 8 && gyroangle <= 12 && CurrentTime >= 1.0) {
                    robot.Intake.setPower(0);
                    state = State.LeftForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftForward4:
                robot.ArmServo.setPosition(0.015);
                robot.Forward(1, 40);
                robot.LiftMotor.setPower(1);
                robot.LiftMotor2.setPower(-1);
                if (!robot.BottomSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                }
                if (robot.DriveDone(40)) {
                    state = State.LeftTurn10;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn10:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    robot.Intake.setPower(0);
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;

            /**End of the Left version of the code**/

            case CenterForward1:
                robot.Forward(1, 20);
                robot.IntakeServo.setPosition(.1);
                if (robot.DriveDone(20)) {
                    state = State.CenterTurn1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.CenterForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterForward2:
                robot.Forward(1, 52);
                if (robot.DriveDone(52)) {
                    state = State.CenterTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn2:
                robot.TurnAbsolute(120, gyroangle);
                if (gyroangle >= 118 && gyroangle <= 122 && CurrentTime >= 1.0) {
                    state = State.CenterForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterForward3:
                robot.Forward(1, 13);
                if (robot.DriveDone(13)) {
                    state = State.CenterTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn3:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137 && CurrentTime >= 1) {
                    state = State.CenterExtendo1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterExtendo1:
                robot.Extendo(1, 30);
                if (robot.DriveDone(13)) {
                    state = State.CenterMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterMarkerDrop:
                robot.IntakeServo.setPosition(1);
                if (CurrentTime  >= 0.5) {
                    time.reset();
                    state = State.CenterMarkerDrop2;
                    robot.Kill();
                }
                break;

            case CenterMarkerDrop2:
                robot.Intake.setPower(.5);
                if(CurrentTime >= 1.0) {
                    state = State.CenterRetracto1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterRetracto1:
                robot.IntakeServo.setPosition(.1);
                robot.Retracto(1, 30);
                if (robot.DriveDone(13)) {
                    state = State.CenterTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn4:
                robot.Intake.setPower(0);
                robot.TurnAbsolute(120, gyroangle);
                if (gyroangle >= 118 && gyroangle <= 122 && CurrentTime >= 1.0) {
                    robot.Intake.setPower(0);
                    state = State.CenterReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterReverse1:
                robot.Reverse(1, 13);
                if (robot.DriveDone(13)) {
                    state = State.CenterTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn5:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.CenterReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterReverse2:
                robot.Reverse(1, 60);
                if (robot.DriveDone(60)) {
                    state = State.CenterTurn6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn6:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    state = State.CenterExtendo2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterExtendo2:
                robot.Extendo(1, 30);
                if (robot.DriveDone(6.5)) {
                    state = State.CenterIntake1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterIntake1:
                robot.IntakeServo.setPosition(1);
                if (CurrentTime >= .2) {
                    state = State.CenterIntake2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterIntake2:
                robot.Intake.setPower(1);
                robot.Extendo(1, 30);
                if (robot.DriveDone(6.5)) {
                    if (CurrentTime >= .8) ;
                    {
                        robot.IntakeServo.setPosition(.1);
                        state = State.CenterRetracto2;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CenterRetracto2:
                robot.Retracto(1, 30);
                if (robot.DriveDone(13)) {
                    robot.Intake.setPower(-1);
                    if (CurrentTime >= 0.5) {
                        state = State.CenterTurn7;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CenterTurn7:
                robot.TurnAbsolute(-10, gyroangle);
                robot.Intake.setPower(-1);
                if (gyroangle >= -12 && gyroangle <= -8 && CurrentTime >= 1.0) {
                    robot.Intake.setPower(0);
                    state = State.CenterReverse3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterReverse3:
                robot.Reverse(1, 15);
                robot.Intake.setPower(-1);
                if (robot.DriveDone(15)) {
                    robot.Intake.setPower(0);
                    state = State.CenterTurn8;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn8:
                robot.TurnAbsolute(0, gyroangle);
                robot.Intake.setPower(-1);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    robot.Intake.setPower(0);
                    state = State.CenterCycleScore1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterCycleScore1:
                robot.ArmServo.setPosition(0.12);
                if (CurrentTime >= 1) {
                    state = State.CenterShake1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterShake1:
                robot.ArmServo.setPosition(.12);
                robot.Forward(1, 15);
                if (robot.DriveDone(1)) {
                    state = State.CenterShake2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterShake2:
                robot.ArmServo.setPosition(.12);
                robot.Reverse(1, 15);
                if (robot.DriveDone(1)) {
                    state = State.CenterTurn9;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn9:
                robot.TurnAbsolute(-25, gyroangle);
                if (gyroangle >= -27 && gyroangle <= -23 && CurrentTime >= 1.0) {
                    state = State.CenterForward5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterForward5:
                robot.Reverse(1, 7);
                if (robot.DriveDone(7)) {
                    state = State.CenterTurn10;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn10:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    state = State.CenterForward6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterForward6:
                robot.Reverse(1, 15);
                if (robot.DriveDone(15)) {
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;

            /**End of the Center version of the code**/

            case RightForward1:
                robot.Forward(1, 20);
                robot.IntakeServo.setPosition(0.1);
                if (robot.DriveDone(20)) {
                    state = State.LeftTurn1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.RightForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightForward2:
                robot.Forward(1, 52);
                if (robot.DriveDone(52)) {
                    state = State.RightTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn2:
                robot.TurnAbsolute(120, gyroangle);
                if (gyroangle >= 118 && gyroangle <= 122 && CurrentTime >= 1.0) {
                    state = State.RightForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightForward3:
                robot.Forward(1, 13);
                if (robot.DriveDone(13)) {
                    state = State.RightTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn3:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137 && CurrentTime >= 1) {
                    state = State.RightExtendo1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightExtendo1:
                robot.IntakeServo.setPosition(1);
                if (CurrentTime  >= 0.5) {
                    time.reset();
                    state = State.RightMarkerDrop2;
                    robot.Kill();
                }
                break;

            case RightMarkerDrop:
                robot.IntakeServo.setPosition(1);
                if (robot.IntakeServo.getPosition() >= 1) {
                    robot.Intake.setPower(1);
                    if (CurrentTime >= 1) {
                        state = State.RightMarkerDrop2;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case RightMarkerDrop2:
                robot.IntakeServo.setPosition(0.1);
                if (robot.IntakeServo.getPosition() >= 0.1) {
                    state = State.RightRetracto1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightRetracto1:
                robot.IntakeServo.setPosition(.1);
                robot.Retracto(1, 30);
                if (robot.DriveDone(13)) {
                    state = State.RightTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn4:
                robot.Intake.setPower(0);
                robot.TurnAbsolute(120, gyroangle);
                if (gyroangle >= 118 && gyroangle <= 122 && CurrentTime >= 1.0) {
                    robot.Intake.setPower(0);
                    state = State.RightReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightReverse1:
                robot.Reverse(1, 13);
                if (robot.DriveDone(13)) {
                    state = State.RightTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn5:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.RightReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightReverse2:
                robot.Reverse(1, 75);
                if (robot.DriveDone(75)) {
                    robot.Intake.setPower(0);
                    state = State.RightTurn6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn6:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    state = State.RightReverse3;
                    time.reset();
                    robot.Kill();
                }

            case RightReverse3:
                robot.Reverse(1, 33);
                if (robot.DriveDone(33)) {
                    robot.Intake.setPower(0);
                    state = State.RightTurn7;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn7:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    state = State.RightExtendo2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightExtendo2:
                robot.Extendo(1, 30);
                if (robot.DriveDone(6.5)) {
                    state = State.RightIntake1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightIntake1:
                robot.IntakeServo.setPosition(1);
                if (CurrentTime >= .5) {
                    state = State.RightIntake2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightIntake2:
                robot.Intake.setPower(1);
                robot.Extendo(1, 6.5);
                if (robot.DriveDone(6.5)) {
                    if (CurrentTime >= 2) {
                        robot.IntakeServo.setPosition(.1);
                        state = State.LeftRetracto2;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case RightRetracto2:
                robot.Retracto(1, 30);
                if (robot.DriveDone(13)) {
                    robot.Intake.setPower(-1);
                    if (CurrentTime >= 0.5) {
                        state = State.RightReverse4;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case RightReverse4:
                robot.Reverse(1, 20);
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                }
                if (robot.DriveDone(20)) {
                    state = State.RightCycleScore1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightCycleScore1:
                robot.ArmServo.setPosition(1);
                if (robot.ArmServo.getPosition() >= 0.12) {
                    state = State.RightShake1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightShake1:
                robot.ArmServo.setPosition(.12);
                robot.Forward(1, 30);
                if (robot.DriveDone(2)) {
                    state = State.RightShake2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightShake2:
                robot.ArmServo.setPosition(.12);
                robot.Reverse(1, 30);
                if (robot.DriveDone(2)) {
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;

            /**End of the Right version of the code**/

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