package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequence.SamplePickSeq;
import org.firstinspires.ftc.teamcode.Sequence.SamplePrePickSeq;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="GetValues", group="TeleOp")
public class GetValues extends LinearOpMode {
    private RobotHardware robot;
    private Intake intake;
    private Outtake outtake;
    private MecanumDrive drive;
    public double botHeading;

    public double multiplier=1;
    public double strafe = 0.7, speed = 0.7, turn = 0.7;
    public static int incr;
    public static List<Action> runningActions = new ArrayList<>();

    public static int lifterPos=0;
    @Override
    public void runOpMode() {

        robot = new RobotHardware();
        robot.init(hardwareMap, telemetry);
        intake = new Intake(robot);
        outtake = new Outtake(robot);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {

        }

        waitForStart();

        while (opModeIsActive()) {
            runningActions = updateAction();
            botHeading = drive.pose.heading.toDouble();


            // TODO =============================================== INIT===========================================================

            if(gamepad1.dpad_up){

                robot.LifterL.setTargetPosition(Globals.LifterIncr);
                robot.LifterR.setTargetPosition(Globals.LifterIncr);

                robot.LifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.LifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.LifterL.setPower(0.5);
                robot.LifterR.setPower(0.5);
            }
            if(gamepad1.dpad_left){
                incr += Globals.ExtensionInc;
                robot.LifterL.setTargetPosition(incr);
                robot.LifterR.setTargetPosition(incr);

                robot.LifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.LifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.LifterL.setPower(0.5);
                robot.LifterR.setPower(0.5);
            }
            if(gamepad1.dpad_right){
                incr -= Globals.ExtensionInc;
                robot.LifterL.setTargetPosition(incr);
                robot.LifterR.setTargetPosition(incr);

                robot.LifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.LifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.LifterL.setPower(0.5);
                robot.LifterR.setPower(0.5);
            }
            if(gamepad1.a){
                intake.updateState(Intake.IntakeRollerState.ON);
            }
            if(gamepad1.b){
                intake.updateState(Intake.IntakeRollerState.OFF);
            }
            if(gamepad1.y){
                intake.updateBucketState(Intake.IntakeBucketState.SAFE);
            }
            if(gamepad1.right_bumper){
                robot.shoulderR.setPosition(Globals.shoulderPos);
                robot.shoulderL.setPosition(1- Globals.shoulderPos);
            }
            if(gamepad1.left_bumper){
//                delivery.SampleInc(0.5);
                robot.sampleGripper.setPosition(Globals.SampleInc);
                }
            if(gamepad1.start){
                runningActions.add(SamplePickSeq.SamplePickSeq(intake, outtake));
            }
            if(gamepad1.back){
                runningActions.add(SamplePrePickSeq.SamplePrePickSeq(intake, outtake)); //sample pre pick
            }
            if(gamepad1.x){
                robot.specimenGripper.setPosition(Globals.SpecimenInc); //sample pre pick
            }

            // TODO ===============================================Field Oriented Drive  ===========================================================

//            if (gamepad1.right_stick_button) {
//                drive.lazyImu.get().resetYaw();
//                drive.navxMicro.initialize();
//                botHeading = drive.pose.heading.toDouble();
//            }
//
//            if (gamepad1.left_trigger > 0) {
//                multiplier = 0.78;
//                drive.driveFieldCentric(Math.pow(Range.clip(-gamepad1.left_stick_x*strafe*multiplier,-1,1),3),
//                        Math.pow(Range.clip(-gamepad1.left_stick_y*speed*multiplier,-1,1),3),
//                        Math.pow(Range.clip(gamepad1.right_stick_x*turn*multiplier,-1,1),3),
//                        botHeading);
//            } else {
//                multiplier=1;
//                drive.driveFieldCentric(Math.pow(Range.clip(-gamepad1.left_stick_x*strafe*multiplier,-1,1),3),
//                        Math.pow(Range.clip(-gamepad1.left_stick_y*speed*multiplier,-1,1),3),
//                        Math.pow(Range.clip(gamepad1.right_stick_x*turn*multiplier,-1,1),3),
//                        botHeading);
//            }
//            drive.updatePoseEstimate();

            // Todo ==================================== Robot Oriented ======================================================================
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(Math.pow(Range.clip(-gamepad1.left_stick_y*speed,-1,1),3),
                                    Math.pow(Range.clip(-gamepad1.left_stick_x*strafe,-1,1),3)),
                            Math.pow(Range.clip(-gamepad1.right_stick_x*turn,-1,1),3))
            );

            if (gamepad1.left_trigger>0.3){
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y*speed*multiplier,-1,1),3),
                                        Math.pow(Range.clip(gamepad1.left_stick_x*strafe*multiplier,-1,1),3)),
                                Math.pow(Range.clip(-gamepad1.right_stick_x*turn*multiplier,-1,1),3))
                );
            }
            // Todo ====================================TELEMETRY======================================================================


            telemetry.addLine("---------------------------");

            telemetry.addData("X Extension Current : ", robot.Extension.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Lifter L Current : ", robot.LifterL.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Lifter R Current : ", robot.LifterR.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine("---------------------------");

            telemetry.addData("Right Back Current : ", drive.rightBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Left Back Current : ", drive.leftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Front Current : ", drive.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Left Front Current : ", drive.leftFront.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine("---------------------------");

            telemetry.addData(" X Extension POS : ", robot.Extension.getCurrentPosition());
            telemetry.addData(" Lifter L POS : ", robot.LifterL.getCurrentPosition());
            telemetry.addData(" Lifter R POS : ", robot.LifterR.getCurrentPosition());

            telemetry.addData(" Bucket POS : ", robot.intakeBucket.getPosition());
            telemetry.addData(" Shoulder L POS : ", robot.shoulderL.getPosition());
            telemetry.addData(" Shoulder R POS : ", robot.shoulderR.getPosition());
//            telemetry.addData(" state : ",Shoulder.shoulderState);

            telemetry.addLine("---------------------------");
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);

            telemetry.update();

        }

    }

    public static List<Action> updateAction(){
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        List<Action> RemovableActions = new ArrayList<>();

        for (Action action : runningActions) {

            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        return newActions;
    }

//    private static List<Action> updateAction() {
//        TelemetryPacket packet = new TelemetryPacket();
//        List<Action> newActions = new ArrayList<>();
//
//        for (Action action : ftc) {
//            if (action.run(packet)) {
//                newActions.add(action);
//            }
//        }
//        return newActions;
//    }




}
