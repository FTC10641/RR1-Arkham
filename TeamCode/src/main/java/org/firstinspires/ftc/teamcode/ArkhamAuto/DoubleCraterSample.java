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

@Autonomous (name = "DoubleCraterSample")

public class DoubleCraterSample extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();
    ArkhamVision vision = new ArkhamVision();

    enum State {
        DoubleCraterDriveOff, DoubleCraterSample, DoubleCraterLiftDown, DoubleCraterCenter,
        DoubleCraterRight, DoubleCraterLeft, LeftKnockOffGold, DoubleCraterLeftTurn1,
        DoubleCraterLeftForward1, DoubleCraterLeftTurn2, DoubleCraterLeftForward2,
        DoubleCraterLeftTurn3, DoubleCraterLeftForward3, DoubleCraterLeftTurn4,
        DoubleCraterLeftForward4, DoubleCraterLeftTurn5, DoubleCraterLeftMarkerDrop,
        DoubleCraterLeftDelay, DoubleCraterDetach, DoubleCraterCenterTurn1,
        DoubleCraterCenterForward1, DoubleCraterCenterKnockOffGold, DoubleCraterCenterReverse,
        DoubleCraterCenterTurn2, DoubleCraterCenterForward2, DoubleCraterCenterTurn3,
        DoubleCraterCenterForward3, DoubleCraterCenterMarkerDrop, DoubleCraterCenterDepotDelay,
        DoubleCraterCenterTurn4, DoubleCraterCenterForward4, DoubleCraterCenterTurn5,
        DoubleCraterCenterForward5, DoubleCraterRightKnockOffGold, DoubleCraterRightReverse1,
        DoubleCraterRightTurn1, DoubleCraterRightForward1, DoubleCraterRightTurn2,
        DoubleCraterRightForward2, DoubleCraterRightTurn3, DoubleCraterRightForward3,
        DoubleCraterRightMarkerDrop, DoubleCraterRightDelay, DoubleCraterRightTurn4,
        DoubleCraterRightForward4, DoubleCraterRightTurn5, DoubleCraterRightForward5, Stop
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
        state = State.DoubleCraterDetach;
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
        telemetry.addData("Current State", state.toString());
        telemetry.update();

        switch (state) {


            case DoubleCraterDetach:
                robot.LiftMotor.setPower(-1);
                robot.LiftMotor2.setPower(1);
                if (!robot.TopSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = State.DoubleCraterDriveOff;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes down and lands**/

            case DoubleCraterDriveOff:
                robot.Forward(1, 2);
                if (robot.DriveDone(2)) {
                    state = State.DoubleCraterLiftDown;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes forward to bring the latch completely off the lander **/

            case DoubleCraterLiftDown:
                robot.LiftMotor.setPower(1);
                robot.LiftMotor2.setPower(-1);
                if (!robot.BottomSwitch.getState() == true) {
                    robot.LiftMotor.setPower(0);
                    robot.LiftMotor2.setPower(0);
                    state = State.DoubleCraterSample;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterSample:
                if (CurrentTime >= 5) {
                    telemetry.addData("Gold Mineral Position", "Center");
                    state = State.DoubleCraterCenter;
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
                                    state = State.DoubleCraterLeft;
                                    left = true;
                                    time.reset();
                                    robot.Kill();
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    state = State.DoubleCraterRight;
                                    time.reset();
                                    right = true;
                                    robot.Kill();
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    state = State.DoubleCraterCenter;
                                    center = true;
                                    time.reset();
                                    robot.Kill();
                                }
                            }
                        }
                    }
                }
                break;

            case DoubleCraterLeft:
                robot.TurnAbsolute(25,gyroangle);
                if (gyroangle>=23 && gyroangle<=27 && CurrentTime >= 1){
                    state = State.LeftKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftKnockOffGold:
                robot.Forward(1, 35);
                robot.Intake.setPower(1);
                if (robot.DriveDone(35)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCraterLeftTurn1 ;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterLeftTurn1:
                robot.TurnAbsolute(90,gyroangle);
                if (gyroangle>=88 && gyroangle<=92 && CurrentTime >= 1){
                    state = State.DoubleCraterLeftForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterLeftForward1:
                robot.Forward(1, 32);
                robot.Intake.setPower(1);
                if (robot.DriveDone(32)) {
                    state = State.DoubleCraterLeftTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterLeftTurn2:
                robot.TurnAbsolute(135,gyroangle);
                if (gyroangle>=133 && gyroangle<=137 && CurrentTime >= 1){
                    state = State.DoubleCraterLeftForward1;
                    time.reset();
                    robot.Kill();
                }
                break;


            case DoubleCraterLeftForward2:
                robot.Forward(1, 64);
                robot.Intake.setPower(1);
                if (robot.DriveDone(64)) {
                    state = State.DoubleCraterLeftMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterLeftMarkerDrop:
                robot.ArmServo.setPosition(0.015);
                robot.RightServo.setPosition(0);
                if( robot.RightServo.getPosition()>=0){
                    state = State.DoubleCraterLeftDelay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterLeftDelay:
                if (CurrentTime >= .5) {
                    robot.RightServo.setPosition(0.95);
                    state = State.DoubleCraterLeftTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterLeftTurn3:
                robot.TurnAbsolute(-67,gyroangle);
                if (gyroangle>=-69 && gyroangle<=-65 && CurrentTime >= 1){
                    robot.RightServo.setPosition(0.95);
                    state = State.DoubleCraterLeftForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterLeftForward3:
                robot.Forward(1, 35);
                if (robot.DriveDone(35)) {
                    state = State.DoubleCraterLeftTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterLeftTurn4:
                robot.TurnAbsolute(0,gyroangle);
                if (gyroangle>=-2 && gyroangle<=2 && CurrentTime >= 1){
                    state = State.DoubleCraterLeftForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterLeftForward4:
                robot.Forward(1, 80);
                if (robot.DriveDone(80)) {
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;
            /**End of the Left version of the code**/


            case DoubleCraterCenter:
                robot.TurnAbsolute(0,gyroangle );
                if (gyroangle>=-2 && gyroangle<=2 && CurrentTime >= 1.0) {
                    state = State.DoubleCraterCenterKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterKnockOffGold:
                robot.Forward(1, 35);
                robot.Intake.setPower(1);
                if (robot.DriveDone(35)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCraterCenterReverse;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterReverse:
                robot.Reverse(1, 10);
                robot.Intake.setPower(1);
                if (robot.DriveDone(10)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCraterCenterTurn1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 2) {
                    state = State.DoubleCraterCenterForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterForward1:
                robot.Forward(1, 45);
                robot.Intake.setPower(1);
                if (robot.DriveDone(45)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCraterCenterTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterTurn2:
                robot.TurnAbsolute(120, gyroangle);
                if (gyroangle >= 118 && gyroangle <= 122 && CurrentTime >= 1) {
                    state = State.DoubleCraterCenterForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterForward2:
                robot.Forward(1, 18);
                robot.Intake.setPower(1);
                if (robot.DriveDone(18)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCraterCenterTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterTurn3:
                robot.TurnAbsolute(135, gyroangle);
                if (gyroangle >= 133 && gyroangle <= 137 && CurrentTime >= 2) {
                    state = State.DoubleCraterCenterForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterForward3:
                robot.Forward(1, 35);
                robot.Intake.setPower(1);
                if (robot.DriveDone(35)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCraterCenterMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterMarkerDrop:
                robot.ArmServo.setPosition(0.015);
                robot.RightServo.setPosition(0);
                if( robot.RightServo.getPosition()>=0){
                    state = State.DoubleCraterCenterDepotDelay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterDepotDelay:
                if (CurrentTime >= .5) {
                    state = State.DoubleCraterCenterTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterTurn4:
                robot.TurnAbsolute(-90, gyroangle);
                if (gyroangle >= -92 && gyroangle <= -88 && CurrentTime >= 2) {
                    state = State.DoubleCraterCenterForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterForward4:
                robot.Forward(1, 35);
                if (robot.DriveDone(35)) {
                    state = State.DoubleCraterCenterTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterTurn5:
                robot.TurnAbsolute(0,gyroangle);
                if (gyroangle>=-2 && gyroangle<=2 && CurrentTime >= 1){
                    state = State.DoubleCraterCenterForward5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterCenterForward5:
                robot.Forward(1, 40);
                if (robot.DriveDone(40)) {
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;
            /**End of the Center version of the code**/


            case DoubleCraterRight:
                robot.TurnAbsolute(-25,gyroangle);
                if (gyroangle>=-27 && gyroangle<=-23 && CurrentTime >= 1){
                    state = State.DoubleCraterRightKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightKnockOffGold:
                robot.Forward(1, 35);
                robot.Intake.setPower(1);
                if (robot.DriveDone(35)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCraterRightReverse1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightReverse1:
                robot.Reverse(1, 12);
                robot.Intake.setPower(1);
                if (robot.DriveDone(12)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCraterRightTurn1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92 && CurrentTime >= 2) {
                    state = State.DoubleCraterRightForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightForward1:
                robot.Forward(1,55);
                robot.Intake.setPower(1);
                if (robot.DriveDone(55)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCraterRightTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightTurn2:
                robot.TurnAbsolute(120, gyroangle);
                if (gyroangle >= 118 && gyroangle <= 122 && CurrentTime >= 2) {
                    state = State.DoubleCraterRightForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightForward2:
                robot.Forward(1, 20);
                robot.Intake.setPower(1);
                if (robot.DriveDone(20)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCraterRightTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightTurn3:
                robot.TurnAbsolute(133, gyroangle);
                if (gyroangle >= 131 && gyroangle <= 135 && CurrentTime >= 1) {
                    state = State.DoubleCraterRightForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightForward3:
                robot.Forward(1, 45);
                robot.Intake.setPower(1);
                if (robot.DriveDone(45)) {
                    robot.Intake.setPower(0);
                    state = State.DoubleCraterRightMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightMarkerDrop:
                robot.ArmServo.setPosition(0.015);
                robot.RightServo.setPosition(0);
                if(robot.RightServo.getPosition()>=0){
                    state = State.DoubleCraterRightDelay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightDelay:
                if (CurrentTime >= .5) {
                    state = State.DoubleCraterRightTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightTurn4:
                robot.TurnAbsolute(-67, gyroangle);
                if (gyroangle >= -69 && gyroangle <= -65 && CurrentTime >= 1) {
                    state = State.DoubleCraterRightForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightForward4:
                robot.Forward(1, 35);
                if (robot.DriveDone(35)) {
                    state = State.DoubleCraterRightTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightTurn5:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 1) {
                    state = State.DoubleCraterRightForward5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case DoubleCraterRightForward5:
                robot.Forward(1, 50);
                if (robot.DriveDone(50)) {
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

    String formatAngle (AngleUnit angleUnit,double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees ( double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    
}