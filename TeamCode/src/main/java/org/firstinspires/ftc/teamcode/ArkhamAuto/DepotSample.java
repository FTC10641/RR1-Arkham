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

@Autonomous(name = "DepotSample")

public class DepotSample extends OpMode {

        ArkhamHW robot = new ArkhamHW();
        ArkhamSensors sensors = new ArkhamSensors();
        ArkhamVision vision = new ArkhamVision();

        enum State {
            DepotDetach, DepotDriveOff, DepotLiftDown, DepotLeft, DepotCenter, DepotRight, DepotCenterKnockOffGold, DepotCenterReverse, DepotSample, DepotDelay, DepotLeftKnockOffGold, DepotLeftTurn1, DepotLeftForward2, LeftReverse, DepotRightKnockOffGold, DepotReverse, DepotStop
        }

        DepotSample.State state;
        ElapsedTime time;
        boolean right = false;
        boolean left = false;
        boolean center = false;

        @Override
        public void init() {
            robot.init(hardwareMap);
            sensors.initSensors(hardwareMap);
            vision.initVision(hardwareMap);
            state = DepotSample.State.DepotDetach;
            time = new ElapsedTime();
            robot.LiftMotor.setPower(.09);
            robot.LiftMotor2.setPower(.09);
        }

        @Override
        public void loop() {
            double CurrentTime = time.time();
            telemetry.addData("time", CurrentTime);
            double gyroangle;
            double lifty;
            sensors.angles = sensors.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            lifty = Double.parseDouble(formatAngle(sensors.angles.angleUnit, sensors.angles.secondAngle));
            gyroangle = Double.parseDouble(formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));
            telemetry.addData("Heading", formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));
            telemetry.addData("Current State", state.toString());
            telemetry.update();


            switch (state) {


                case DepotDetach:
                    robot.LiftMotor.setPower(-1);
                    robot.LiftMotor2.setPower(1);
                    if (!robot.TopSwitch.getState() == true) {
                        robot.LiftMotor.setPower(0);
                        robot.LiftMotor2.setPower(0);
                        state = DepotSample.State.DepotDriveOff;
                        time.reset();
                        robot.Kill();
                    }
                    break;
                /** The robot goes down and lands**/

                case DepotDriveOff:
                    robot.Forward(1, 2);
                    if (robot.DriveDone(2)) {
                        state = DepotSample.State.DepotLiftDown;
                        time.reset();
                        robot.Kill();
                    }
                    break;
                /** The robot goes forward to bring the latch completely off the lander **/

                case DepotLiftDown:
                    robot.LiftMotor.setPower(1);
                    robot.LiftMotor2.setPower(-1);
                    if (!robot.BottomSwitch.getState() == true) {
                        robot.LiftMotor.setPower(0);
                        robot.LiftMotor2.setPower(0);
                        state = State.DepotSample;
                        time.reset();
                        robot.Kill();
                    }
                    break;
                /** the lift goes back down**/


                case DepotSample:
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
                                        state = DepotSample.State.DepotLeft;
                                        left = true;
                                        time.reset();
                                        robot.Kill();
                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                        telemetry.addData("Gold Mineral Position", "Right");
                                        state = DepotSample.State.DepotRight;
                                        time.reset();
                                        right = true;
                                        robot.Kill();
                                    } else {
                                        telemetry.addData("Gold Mineral Position", "Center");
                                        state = DepotSample.State.DepotCenter;
                                        center = true;
                                        time.reset();
                                        robot.Kill();
                                    }
                                }
                            }
                        }
                    }
                    break;

                case DepotLeft:
                    robot.TurnAbsolute(25, gyroangle);
                    if (gyroangle >= 23 && gyroangle <= 27) {
                        state = DepotSample.State.DepotLeftKnockOffGold;
                        time.reset();
                        robot.Kill();
                    }
                    break;

                case DepotCenter:
                    robot.TurnAbsolute(0, gyroangle);
                    if (gyroangle >= -2 && gyroangle <= 2) {
                        state = State.DepotCenterKnockOffGold;
                        time.reset();
                        robot.Kill();
                    }
                    break;

                case DepotRight:
                    robot.TurnAbsolute(-25, gyroangle);
                    if (gyroangle >= -27 && gyroangle <= -23) {
                        state = DepotSample.State.DepotRightKnockOffGold;
                        time.reset();
                        robot.Kill();
                    }
                    break;

                case DepotLeftKnockOffGold:
                    robot.Forward(1, 35);
                    robot.Intake.setPower(-1);
                    if (robot.DriveDone(35)) {
                        state = State.DepotLeftTurn1;
                        robot.Intake.setPower(0);
                        time.reset();
                        robot.Kill();
                    }
                    break;

                case DepotLeftTurn1:
                    robot.TurnAbsolute(-45, gyroangle);
                    if (gyroangle >= -47 && gyroangle <= -43){
                        state = State.DepotLeftForward2;
                        time.reset();
                        robot.Kill();
                    }
                    break;

                case DepotLeftForward2:
                    robot.Forward(1, 20);
                    if (robot.DriveDone(20)){
                        state = State.DepotDelay;
                        time.reset();
                        robot.Kill();
                    }
                    break;

                case DepotCenterKnockOffGold:
                    robot.Forward(1, 35);
                    robot.Intake.setPower(-1);
                    if (robot.DriveDone(35)) {
                        robot.Intake.setPower(0);
                        time.reset();
                        robot.Kill();
                    }
                    break;

                case DepotRightKnockOffGold:
                    robot.Forward(1, 35);
                    robot.Intake.setPower(-1);
                    if (robot.DriveDone(35)) {
                        robot.Intake.setPower(0);
                        time.reset();
                        robot.Kill();
                    }
                    break;

                case DepotDelay:
                    if (CurrentTime >= 2.0) {
                        state = DepotSample.State.DepotReverse;
                        time.reset();
                        robot.Kill();
                    }
                    break;

                case DepotStop:
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