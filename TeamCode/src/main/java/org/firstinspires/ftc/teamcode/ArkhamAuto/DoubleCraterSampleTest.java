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


@Autonomous(name = "DoubleCraterSampleTest")


public class DoubleCraterSampleTest extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();
    ArkhamVision vision = new ArkhamVision();

    enum State {
        Detach,DriveOff,LiftDown,Left,Center,Right, CenterKnockOffGold,CenterReverse, CenterTurn1,
        CenterForward1, CenterTurn2, CenterForward2, Sample, Backup, LeftTurn1, LeftTurn2,
        LeftForward1, Delay, Reverse, LeftForward2, LeftKnockOffGold, RightKnockOffGold, MakerDrop,
        RightReverse1, RightTurn1, RigtForward1, RightTurn2, RightForward2, Direction, MarkerDrop,
        CraterLiftDown, CraterDriveOff, CraterDetach,
        CraterLeft, CraterCenter, CraterRight, CraterSample, LeftMarkerDrop, LeftDepotDelay, DepotLeftReverse,
        CenterMarkerDrop, CenterDepotDelay, LeftCraterDelay, CraterLeftReverse, CraterCenterReverse, RightForward1,
        RightMarkerDrop, RightDelay, RightReverse2, CraterCenterReverse2, RightTurn3, LeftTurn3, LeftForward3, LeftTurn4, LeftForward4, RightForward3, RightTurn4, RightForward4, LeftTurn5, LeftForward5, CenterTurn3, CenterForward3, CenterTurn4, CenterForward4, CenterTurn5, CenterForward5, Stop
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
        robot.LiftMotor.setPower(.09);
        robot.LiftMotor2.setPower(.09);
        robot.RightServo.setPosition(0.85);


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
                robot.Forward(1, 2);
                if (robot.DriveDone(2)) {
                    state = State.CraterLiftDown;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** The robot goes forward to bring the latch completely off the lander **/

            case CraterLiftDown:
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
                if (CurrentTime >= 5) {
                    telemetry.addData("Gold Mineral Position", "Center");
                    state = State.CraterCenter;
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
                                    state = State.CraterLeft;
                                    left = true;
                                    time.reset();
                                    robot.Kill();
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    state = State.CraterRight;
                                    time.reset();
                                    right = true;
                                    robot.Kill();
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    state = State.CraterCenter;
                                    center = true;
                                    time.reset();
                                    robot.Kill();
                                }
                            }
                        }
                    }
                }
                break;


            case CraterLeft:
                robot.TurnAbsolute(25,gyroangle);
                if (gyroangle>=23 && gyroangle<=27){
                    state = State.LeftKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CraterCenter:
                robot.TurnAbsolute(0,gyroangle);
                if (gyroangle>=-2 && gyroangle<=2) {
                    state = State.CenterKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CraterRight:
                robot.TurnAbsolute(-25,gyroangle);
                if (gyroangle>=-27 && gyroangle<=-23){
                    state = State.RightKnockOffGold;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftKnockOffGold:
                robot.Forward(1, 35);
                robot.Intake.setPower(1);
                if (robot.DriveDone(35)) {
                    robot.Intake.setPower(0);
                    state = State.LeftTurn1 ;
                    time.reset();
                    robot.Kill();
                }
                break;
            /** This State makes the robot drive forward to knock off the gold while the intake is going inward so the gold doesn't get in the way **/

            case LeftTurn1:
                robot.TurnAbsolute(90, gyroangle);
                if (gyroangle >= 88 && gyroangle <= 92) {
                    state = State.LeftForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftForward1:
                robot.Forward(1, 32);
                if (robot.DriveDone(32)) {
                    state = State.LeftTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn2:
                robot.TurnAbsolute(131, gyroangle);
                if (gyroangle >= 129 && gyroangle <= 133) {
                    state = State.LeftForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftForward2:
                robot.Forward(1, 60);
                if (robot.DriveDone(60)) {
                    state = State.LeftMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;
            /**End of the Left version of the code**/

            case LeftMarkerDrop:
                robot.ArmServo.setPosition(0.015);
                robot.RightServo.setPosition(0);
                if( robot.RightServo.getPosition()>=0){
                    state = State.LeftCraterDelay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftCraterDelay:
                if (CurrentTime >= .5) {
                    state = State.LeftTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn3:
                robot.TurnAbsolute(-111, gyroangle);
                if (gyroangle >= -113 && gyroangle <= -109) {
                    state = State.LeftForward3;
                    time.reset();
                    robot.Kill();
                }
                break;


            case LeftForward3:
                robot.Forward(1, 35);
                robot.Intake.setPower(1);
                if (robot.DriveDone(35)) {
                    robot.Intake.setPower(0);
                    state = State.LeftTurn4 ;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn4:
                robot.TurnAbsolute(179, gyroangle);
                if (gyroangle >= 177 && gyroangle <= 181) {
                    state = State.LeftForward4;
                    time.reset();
                    robot.Kill();
                }
                break;



            case LeftForward4:
                robot.Forward(1, 12);
                if (robot.DriveDone(12)) {
                    state = State.LeftTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftTurn5:
                robot.TurnAbsolute(-135, gyroangle);
                if (gyroangle >= -137 && gyroangle <= -133) {
                    state = State.LeftForward5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case LeftForward5:
                robot.Forward(1, 45);
                if (robot.DriveDone(45)) {
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;



            case CenterKnockOffGold:
                robot.Forward(1, 38);
                robot.Intake.setPower(1);
                if (robot.DriveDone(38)) {
                    robot.Intake.setPower(0);
                    state = State.CenterReverse;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterReverse:
                robot.Reverse(1, 12);
                if (robot.DriveDone(12)) {
                    state = State.CenterTurn1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn1:
                robot.TurnAbsolute(86, gyroangle);
                if (gyroangle >= 84 && gyroangle <= 88) {
                    state = State.CenterForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterForward1:
                robot.Forward(1, 52);
                if (robot.DriveDone(52)) {
                    state = State.CenterTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn2:
                robot.TurnAbsolute(131, gyroangle);
                if (gyroangle >= 129 && gyroangle <= 133) {
                    state = State.CenterForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterForward2:
                robot.Forward(1, 45);
                if (robot.DriveDone(45)) {
                    state = State.CenterMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;
            /**End of the Center version of the code**/

            case CenterMarkerDrop:
                robot.ArmServo.setPosition(0.015);
                robot.RightServo.setPosition(0);
                if( robot.RightServo.getPosition()>=0){
                    state = State.CenterDepotDelay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterDepotDelay:
                if (CurrentTime >= .5) {
                    state = State.CenterTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn3:
                robot.TurnAbsolute(-92, gyroangle);
                if (gyroangle >= -94 && gyroangle <= -90) {
                    state = State.CenterForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterForward3:
                robot.Forward(1, 36);
                robot.Intake.setPower(1);
                if (robot.DriveDone(36)) {
                    robot.Intake.setPower(0);
                    state = State.CenterTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn4:
                robot.TurnAbsolute(0, gyroangle);
                if (gyroangle >= -2 && gyroangle <= 2) {
                    state = State.CenterForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterForward4:
                robot.Forward(1, 40);
                if (robot.DriveDone(40)) {
                    state = State.CenterTurn5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterTurn5:
                robot.TurnAbsolute(-45, gyroangle);
                if (gyroangle >= -47 && gyroangle <= -43) {
                    state = State.CenterForward5;
                    time.reset();
                    robot.Kill();
                }
                break;

            case CenterForward5:
                robot.Forward(1, 10);
                if (robot.DriveDone(10)) {
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;


            case RightKnockOffGold:
                robot.Forward(1, 35);
                robot.Intake.setPower(1);
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
                robot.TurnAbsolute(85, gyroangle);
                if (gyroangle >= 83 && gyroangle <= 87) {
                    state = State.RightForward1;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightForward1:
                robot.Forward(1, 69);
                if (robot.DriveDone(69)) {
                    state = State.RightTurn2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn2:
                robot.TurnAbsolute(131, gyroangle);
                if (gyroangle >= 129 && gyroangle <= 133) {
                    state = State.RightForward2;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightForward2:
                robot.Forward(1, 45);
                if (robot.DriveDone(45)) {
                    state = State.RightMarkerDrop;
                    time.reset();
                    robot.Kill();
                }
                break;
            /**End of the Right version of the code**/

            case RightMarkerDrop:
                robot.ArmServo.setPosition(0.015);
                robot.RightServo.setPosition(0);
                if( robot.RightServo.getPosition()>=0){
                    state = State.RightDelay;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightDelay:
                if (CurrentTime >= .5) {
                    state = State.RightTurn3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn3:
                robot.TurnAbsolute(-67.5, gyroangle);
                if (gyroangle >= -69.5 && gyroangle <= -65.5) {
                    state = State.RightForward3;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightForward3:
                robot.Forward(1, 35);
                if (robot.DriveDone(35)) {
                    state = State.RightTurn4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightTurn4:
                robot.TurnAbsolute(-22.5, gyroangle);
                if (gyroangle >= -24.5 && gyroangle <= -20.5) {
                    state = State.RightForward4;
                    time.reset();
                    robot.Kill();
                }
                break;

            case RightForward4:
                robot.Forward(1, 40);
                if (robot.DriveDone(40)) {
                    state = State.Stop;
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
