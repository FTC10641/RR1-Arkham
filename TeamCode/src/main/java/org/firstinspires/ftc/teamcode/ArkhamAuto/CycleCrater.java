
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


@Autonomous(name = "CycleCrater")


public class CycleCrater extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();
    ArkhamVision vision = new ArkhamVision();

    enum State {
        CycleCraterDetach, CycleCraterDriveOff, CycleCraterLiftDown,
        CycleCraterSample, CycleCraterLeft, CycleCraterCenter, CycleCraterRight, CycleRightKnockOffGold, CycleCenterKnockOffGold, CycleLeftKnockOffGold, CycleCraterLeftTurn1, CycleCraterLeftForward1, CycleCraterLeftTurn2, CycleCraterLeftForward2, CycleCraterLeftTurn3, CycleCraterLeftForward3, CycleCraterLeftReverse1, CycleCraterLeftTurn4, CycleCraterLeftReverse2, CycleCraterLeftTurn5, LeftRetracto1, LeftExtendo1, LeftExtendo2, CycleCraterLeftReverse3,  CycleScore1, CycleCraterLeftForward4, CycleScore, Stop
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
        state = State.CycleCraterDetach;
        ElapsedTime runtime = new ElapsedTime();
        robot.LiftMotor.setPower(.2);
        robot.LiftMotor2.setPower(.2);
        robot.RightServo.setPosition(0.95);
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
                    state = State.CycleCraterCenter;
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
                                    state = State.CycleCraterLeft;
                                    left = true;
                                    time.reset();
                                    robot.Kill();
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    state = State.CycleCraterRight;
                                    time.reset();
                                    right = true;
                                    robot.Kill();
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    state = State.CycleCraterCenter;
                                    center = true;
                                    time.reset();
                                    robot.Kill();
                                }
                            }
                        }
                    }
                }
                break;


            case CycleCraterLeft:
                robot.TurnAbsolute(27,gyroangle);
                if (gyroangle>=25 && gyroangle<=29 && CurrentTime >= 1.0){
                    state = State.CycleLeftKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleLeftKnockOffGold:
                robot.Forward(1, 25);
                if (robot.DriveDone(25)) {
                    state = State.CycleCraterLeftTurn1 ;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** This State makes the robot drive forward to knock off the gold while the intake is going inward to pick it up**/

            case CycleCraterLeftTurn1:
                        robot.TurnAbsolute(90, gyroangle);
                        if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                            state = State.CycleCraterLeftForward1;
                            time.reset();
                            robot.Kill();
                        }
                        break;

            case CycleCraterLeftForward1:
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
                            state = State.CycleCraterLeftForward2;
                            time.reset();
                            robot.Kill();
                        }
                        break;

            case CycleCraterLeftForward2:
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
                            state = State.CycleCraterLeftForward3;
                            time.reset();
                            robot.Kill();
                        }
                        break;

            case CycleCraterLeftForward3:
                robot.Forward(1, 18);
                if (robot.DriveDone(18)) {
                    state = State.LeftExtendo1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftExtendo1:
                robot.Extendo.setPower(1);
                if (!robot.OutSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower(1);
                    if (CurrentTime >= 0.5) {
                        state = State.LeftRetracto1;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case LeftRetracto1:
                robot.Extendo.setPower(-1);
                robot.Intake.setPower (1);
                if (!robot.InSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower (0);
                    state = State.CycleCraterLeftReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftReverse1:
                robot.Reverse(1, 18);
                robot.Intake.setPower(1);
                if (robot.DriveDone(18)) {
                    robot.Intake.setPower(0);
                    state = State.CycleCraterLeftTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftTurn4:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 1.0) {
                    state = State.CycleCraterLeftReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftReverse2:
                robot.Reverse(1, 25);
                if (robot.DriveDone(25)) {
                    state = State.CycleCraterLeftTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleCraterLeftTurn5:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1.0) {
                    state = State.LeftExtendo2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftExtendo2:
                robot.Extendo.setPower(1);
                if (!robot.OutSwitch.getState() == true) {
                    robot.Extendo.setPower(0);
                    robot.Intake.setPower(1);
                    if (CurrentTime >= 0.5) {
                        state = State.CycleCraterLeftReverse3;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleCraterLeftReverse3:
                robot.Reverse(1, 15);
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                }
                if (robot.DriveDone(15)) {
                    state = State.CycleScore;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CycleScore:
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
                        state = State.CycleCraterLeftForward4;
                        time.reset();
                        robot.Kill();
                    }
                }
                break;

            case CycleCraterLeftForward4:
                robot.Forward(1, 40);
                if (robot.DriveDone(40)) {
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;

            /**End of the Left version of the code**/

            case CycleCraterCenter:
                robot.TurnAbsolute(0,gyroangle );
                if (gyroangle>=-2 && gyroangle<=2 && CurrentTime >= 1.0) {
                    state = State.CycleCenterKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            /**End of the Center version of the code**/

            case CycleCraterRight:
                robot.TurnAbsolute(-23,gyroangle);
                if (gyroangle>=-25 && gyroangle<=-21 && CurrentTime >= 1.0){
                    state = State.CycleRightKnockOffGold;
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
    String formatAngle (AngleUnit angleUnit,double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees ( double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}