package org.firstinspires.ftc.teamcode.ArkhamAuto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.ArkhamTele.ArkhamHW;
import org.firstinspires.ftc.teamcode.ArkhamTele.ArkhamSensors;
import org.firstinspires.ftc.teamcode.ArkhamTele.ArkhamVision;

import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.ArkhamTele.ArkhamVision.LABEL_GOLD_MINERAL;


@Autonomous(name = "ArkhamSample")


public class ArkhamSample extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();
    ArkhamVision vision = new ArkhamVision();
    enum State {
        Start,Forward,left,center,right, Sample, Stop}

    State state;
    ElapsedTime time;

    @Override
    public void init() {
        robot.init(hardwareMap);
        sensors.initSensors(hardwareMap);
        vision.initVision(hardwareMap);
        state = State.Start;
        time = new ElapsedTime();
        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

    }

    @Override
    public void loop() {
        double CurrentTime = time.time();
        telemetry.addData("time", CurrentTime);
        double gyroangle;
        sensors.angles = sensors.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroangle = Double.parseDouble(formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));
        telemetry.addData("Heading", formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));

        switch (state) {

            case Start:
                robot.Kill();
                if (robot.IsBusy()) {
                    state = State.Sample;
                    time.reset();
                }
                break;

            case Sample:
                if (vision.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = vision.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
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
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    state = State.left;
                                    time.reset();
                                    robot.Kill();
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    state = State.right;
                                    time.reset();
                                    robot.Kill();
                                } else {
                                    state = State.center;
                                    time.reset();
                                    robot.Kill();
                                }
                            }
                        }
                    }}
                        break;

                        case left:
                            robot.TurnAbsolute(25, gyroangle);
                            if (gyroangle >= 23 && gyroangle <= 27 && CurrentTime >= 2) {
                                state = State.Forward;
                                time.reset();
                                robot.Kill();
                            }
                            break;

                        case center:
                            robot.TurnAbsolute(0, gyroangle);
                            if (gyroangle >= -2 && gyroangle <= 2 && CurrentTime >= 2) {
                                state = State.Forward;
                                time.reset();
                                robot.Kill();
                            }
                            break;

                        case right:
                            robot.TurnAbsolute(-25, gyroangle);
                            if (gyroangle >= -27 && gyroangle <= -23 && CurrentTime >= 2) {
                                state = State.Forward;
                                time.reset();
                                robot.Kill();
                            }
                            break;

                        case Forward:
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
                            vision.Disable();
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
