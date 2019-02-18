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

@Autonomous(name = "DoubleDepotSample2")

public class DoubleDepotSample2 extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();
    ArkhamVision vision = new ArkhamVision();

    enum State {
        CenterMarkerDrop, CenterDepotDelay, LeftMarkerTurn2, DepotStop, DoubleDepotLiftDown,
        DoubleDepotSample, DoubleDepotRight, DoubleDepotLeft, DoubleDepotCenter, DoubleDepotLeftKnockOffGold, DoubleDepotLeftTurn1,
        DoubleDepotLeftForward2, DoubleLeftMarkerTurn1, DoubleDepotLeftForward3, DoubleDepotDetach, DoubleDepotRightKnockOffGold,
        DoubleDepotStop, DoubleDepotRightTurn1, DoubleDepotRightForward2, DoubleDepotRightMarkerDrop, DoubleDepotRightDelay,
        DoubleDepotRightTurn2, DoubleDepotRightForward3, DoubleDepotRightTurn3, DoubleDepotRightForward4, DoubleDepotCenterForward2,
        DoubleDepotCenterTurn2, DoubleDepotCenterForward1, DoubleDepotCenterTurn1, DoubleDepotCenterReverse, DoubleCenterDepotDelay,
        DoubleCenterMarkerDrop, DoubleDepotCenterKnockOffGold, DoubleDepotLeftMarkerTurn2, DoubleDepotLeftDelay, DoubleDepotLeftMarkerDrop,
        DoubleDepotLeftTurn2, DoubleDepotLeftForward4, DoubleDepotLeftTurn3, DoubleDepotLeftForward5, DoubleDepotLeftTurn4, DoubleDepotLeftForward6,
        DoubleDepotCenterTurn3, DoubleDepotCenterForward3, DoubleDepotCenterTurn4, DoubleDepotCenterForward4, DoubleDepotLeftReverse1, DoubleDepotLeftReverse2, DoubleDepotDriveOff
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
        state = DoubleDepotSample2.State.DoubleDepotDetach;
        time = new ElapsedTime();
        robot.LiftMotor.setPower(.09);
        robot.LiftMotor2.setPower(.09);
        robot.RightServo.setPosition(0.85);
        robot.ArmServo.setPosition(0.015);
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


            case DoubleDepotDetach:
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = State.DoubleDepotDriveOff;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes down and lands**/

            case DoubleDepotDriveOff:
                robot.Forward(1, 2);
                if (robot.DriveDone(2)) {
                    state = State.DoubleDepotLiftDown;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes forward to bring the latch completely off the lander **/

            case DoubleDepotLiftDown:
                robot.LiftMotor.setPower(1);
                robot.LiftMotor2.setPower(-1);
                if (!robot.BottomSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = State.DoubleDepotSample;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The lift goes back down**/


            case DoubleDepotSample:
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
                                    state = State.DoubleDepotLeft;
                                    left = true;
                                    time.reset();
                                    robot.Kill();
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    state = State.DoubleDepotRight;
                                    time.reset();
                                    right = true;
                                    robot.Kill();
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    state = State.DoubleDepotCenter;
                                    center = true;
                                    time.reset();
                                    robot.Kill();
                                }
                            }
                        }
                    }
                }
                break;

            case DoubleDepotLeft:
                robot.TurnAbsolute(23, gyroangle);
                if (gyroangle >= 21 && gyroangle <= 25) {
                    state = State.DoubleDepotLeftKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftKnockOffGold:
                robot.Forward(1, 56);
                robot.Intake.setPower(1);
                if (robot.DriveDone(56)) {
                    state = State.DoubleDepotLeftTurn1;
                    robot.Intake.setPower(0);
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftTurn1:
                robot.TurnAbsolute(-45, gyroangle);
                if (gyroangle >= -47 && gyroangle <= -43 && CurrentTime > 2){
                    state = State.DoubleDepotLeftForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftForward2:
                robot.Forward(1, 20);
                robot.Intake.setPower(1);
                if (robot.DriveDone(20)){
                    state = State.DoubleLeftMarkerTurn1;
                    robot.Intake.setPower(0);
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleLeftMarkerTurn1:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2) {
                    state = State.DoubleDepotLeftMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftMarkerDrop:
                robot.ArmServo.setPosition(0.015);
                robot.RightServo.setPosition(0);
                if( robot.RightServo.getPosition()>=0){
                    state = State.DoubleDepotLeftDelay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftDelay:
                if (CurrentTime >= .5) {
                    state = State.DoubleDepotLeftMarkerTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftMarkerTurn2:
                robot.TurnAbsolute(-45, gyroangle);
                if (gyroangle >= -47 && gyroangle <= -43 && CurrentTime > 1) {
                    robot.RightServo.setPosition(0.85);
                    state = State.DoubleDepotLeftReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftReverse1:
                robot.Reverse(1, 20);
                robot.Intake.setPower(1);
                if (robot.DriveDone(20)) {
                    state = State.DoubleDepotLeftTurn2;
                    robot.Intake.setPower(0);
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftTurn2:
                robot.TurnAbsolute(23, gyroangle);
                if (gyroangle >= 21 && gyroangle <= 25 && CurrentTime > 2){
                    state = State.DoubleDepotLeftReverse2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftReverse2:
                robot.Reverse(1, 40);
                robot.Intake.setPower(1);
                if (robot.DriveDone(40)) {
                    state = State.DoubleDepotLeftTurn3;
                    robot.Intake.setPower(0);
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftTurn3:
                robot.TurnAbsolute(-90, gyroangle);
                if (gyroangle >= -92 && gyroangle <= -88){
                    state = State.DoubleDepotLeftForward5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftForward5:
                robot.Forward(1, 30);
                robot.Intake.setPower(1);
                if (robot.DriveDone(30)) {
                    state = State.DoubleDepotStop;
                    robot.Intake.setPower(0);
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftTurn4:
                robot.TurnAbsolute(-177, gyroangle);
                if (gyroangle >= -179 && gyroangle <= -175){
                    state = State.DoubleDepotLeftForward6;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotLeftForward6:
                robot.Forward(1, 30);
                robot.Intake.setPower(1);
                if (robot.DriveDone(30)) {
                    state = State.DoubleDepotStop;
                    robot.Intake.setPower(0);
                    time.reset();
                    robot.Kill();
                }
                break;





            /**End of the Left version of auto**/

            case DoubleDepotCenter:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2) {
                    state = State.DoubleDepotCenterKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotCenterKnockOffGold:
                robot.Forward(1, 60);
                robot.Intake.setPower(1);
                if (robot.DriveDone(60)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCenterMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCenterMarkerDrop:
                robot.ArmServo.setPosition(0.015);
                robot.RightServo.setPosition(0);
                if( robot.RightServo.getPosition()>=0){
                    state = State.DoubleCenterDepotDelay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCenterDepotDelay:
                robot.RightServo.setPosition(0.5);
                if (CurrentTime >= .5) {
                    state = State.DoubleDepotCenterReverse;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotCenterReverse:
                robot.Reverse(1, 7);
                robot.Intake.setPower(1);
                if (robot.DriveDone(7)) {
                    robot.Intake.setPower(0);
                    robot.RightServo.setPosition(0.85);
                    state = State.DoubleDepotCenterTurn1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotCenterTurn1:
                robot.TurnAbsolute(-90, gyroangle);
                if (gyroangle >= -92 && gyroangle <= -88){
                    state = State.DoubleDepotCenterForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotCenterForward1:
                robot.Forward(1, 25);
                robot.Intake.setPower(1);
                if (robot.DriveDone(25)){
                    state = State.DoubleDepotCenterTurn2;
                    robot.Intake.setPower(0);
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotCenterTurn2:
                robot.TurnAbsolute(-135, gyroangle);
                if (gyroangle >= -137 && gyroangle <= -133){
                    state = State.DoubleDepotCenterForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotCenterForward2:
                robot.Forward(1, 30);
                robot.Intake.setPower(1);
                if (robot.DriveDone(30)){
                    state = State.DoubleDepotCenterTurn3;
                    robot.Intake.setPower(0);
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotCenterTurn3:
                robot.TurnAbsolute(-177, gyroangle);
                if (gyroangle >= -179 && gyroangle <= -175){
                    state = State.DoubleDepotCenterForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotCenterForward3:
                robot.Forward(1, 40);
                robot.Intake.setPower(1);
                if (robot.DriveDone(40)){
                    state = State.DoubleDepotCenterTurn4;
                    robot.Intake.setPower(0);
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotCenterTurn4:
                robot.TurnAbsolute(-90, gyroangle);
                if (gyroangle >= -92 && gyroangle <= -88){
                    state = State.DoubleDepotCenterForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotCenterForward4:
                robot.Forward(1, 20);
                robot.Intake.setPower(1);
                if (robot.DriveDone(20)){
                    state = State.DoubleDepotStop;
                    robot.Intake.setPower(0);
                    time.reset();
                    robot.Kill();
                }
                break;

            /**End of the Center version of auto**/

            case DoubleDepotRight:
                robot.TurnAbsolute(-25, gyroangle);
                if (gyroangle >= -27 && gyroangle <= -23) {
                    state = State.DoubleDepotRightKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotRightKnockOffGold:
                robot.Forward(1, 52);
                robot.Intake.setPower(1);
                if (robot.DriveDone(52)) {
                    state = State.DoubleDepotRightTurn1;
                    robot.Intake.setPower(0);
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotRightTurn1:
                robot.TurnAbsolute(43, gyroangle);
                if (gyroangle >= 41 && gyroangle <= 45){
                    state = State.DoubleDepotRightForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotRightForward2:
                robot.Forward(1, 20);
                if (robot.DriveDone(20)){
                    state = State.DoubleDepotRightMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotRightMarkerDrop:
                robot.ArmServo.setPosition(0.015);
                robot.RightServo.setPosition(0);
                if( robot.RightServo.getPosition()>=0){
                    state = State.DoubleDepotRightDelay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotRightDelay:
                if (CurrentTime >= .5) {
                    state = State.DoubleDepotRightTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotRightTurn2:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92) {
                    robot.RightServo.setPosition(0.85);
                    state = State.DoubleDepotRightForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotRightForward3:
                robot.Forward(1, 15);
                if (robot.DriveDone(15)){
                    state = State.DoubleDepotRightTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotRightTurn3:
                robot.TurnAbsolute(133, gyroangle);
                if (gyroangle >= 131 && gyroangle <= 135) {
                    state = State.DoubleDepotRightForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleDepotRightForward4:
                robot.Forward(1, 65);
                if (robot.DriveDone(65)){
                    state = State.DoubleDepotStop;
                    time.reset();
                    robot.Kill();
                }
                break;
            /**End of the Right version of auto**/

            case DoubleDepotStop:
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