package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequence.HangSeq;
import org.firstinspires.ftc.teamcode.Sequence.InitSeq;
import org.firstinspires.ftc.teamcode.Sequence.PreHangSeq;
import org.firstinspires.ftc.teamcode.Sequence.PreSpeciDropSeq;
import org.firstinspires.ftc.teamcode.Sequence.SampleDrop;
import org.firstinspires.ftc.teamcode.Sequence.SamplePickAgainSeq;
import org.firstinspires.ftc.teamcode.Sequence.SamplePickSeq;
import org.firstinspires.ftc.teamcode.Sequence.SamplePrePickSeq;
import org.firstinspires.ftc.teamcode.Sequence.SampleTransferSeq;
import org.firstinspires.ftc.teamcode.Sequence.SpeciDropSeq;
import org.firstinspires.ftc.teamcode.Sequence.SpeciPickSeq;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Sankalp", group="TeleOp")
public class Tele extends LinearOpMode {
    private RobotHardware robot;
    private Intake intake;
    private Outtake outtake;
    private MecanumDrive drive;
    public double botHeading;

    public double multiplier=1;
    public double strafe = 0.95, speed = 0.95, turn = 0.95;

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
            Actions.runBlocking(
                    new SequentialAction(
                            intake.IntakeGripperCommands(Intake.SampleGripperState.INIT),
                            intake.IntakeBucketCommands(Intake.IntakeBucketState.PASS),
                            intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF),
                            outtake.LifterCommands(Outtake.LifterState.INIT),
                            outtake.ShoulderCommands(Outtake.ShoulderState.INIT),
                            intake.ExtensionCommands(Intake.ExtensionState.INIT)
                    )                          );

        }
        robot.resetEncoder();
        waitForStart();

        while (opModeIsActive()) {
            runningActions = updateAction();
            botHeading = drive.pose.heading.toDouble();


            // TODO =============================================== INIT===========================================================


            if(gamepad1.dpad_left){
                runningActions.add(SpeciPickSeq.SpeciPickAction(intake, outtake)); //sample pre pick
            }
            if(gamepad1.dpad_up){
                runningActions.add(PreSpeciDropSeq.PreSpeciDropSeq(intake, outtake)); //sample pre pick
            }
            if(gamepad1.dpad_down){
                runningActions.add(SpeciDropSeq.SpeciDropSeq(intake, outtake)); //sample pre pick
            }
            if(gamepad1.dpad_right){
                runningActions.add(PreHangSeq.PreHangSeq(intake, outtake)); //sample pre pick
            }
            if(gamepad1.left_trigger>0)
            {
                runningActions.add(HangSeq.HangSeq(intake, outtake)); //sample pre pick
            }

            if(gamepad1.a){
                runningActions.add(SamplePrePickSeq.SamplePrePickSeq(intake, outtake)); //sample pre pick
            }
            if(gamepad1.x){
                runningActions.add(SamplePickSeq.SamplePickSeq(intake, outtake));
            }
            if(gamepad1.b){
                runningActions.add(SampleDrop.Drop(intake, outtake));
            }
            if(gamepad1.y){
                runningActions.add(SampleTransferSeq.TransferSeq(intake, outtake));
            }
            if(gamepad1.start){
                runningActions.add(SampleDrop.Release(intake, outtake));
            }
            if(gamepad1.back){
                runningActions.add(SampleDrop.OFF(intake, outtake));
            }

            if(gamepad1.right_bumper){
                runningActions.add(InitSeq.InitSeqence(intake, outtake));
            }
            if(gamepad1.left_bumper){
                runningActions.add(SampleDrop.PreDrop(intake, outtake));
            }
//            if(gamepad1.start){
//                runningActions.add(SamplePickSeq.SamplePickSeq(intake, outtake));
//            }
//            if(gamepad1.back){
//                runningActions.add(SamplePrePickSeq.SamplePrePickSeq(intake, outtake)); //sample pre pick
//            }

            if(gamepad1.right_trigger>0)
            {
                runningActions.add(SamplePickSeq.Extension(intake, outtake)); //sample pre pick
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
            telemetry.addData(" Sample Gripper POS : ", robot.sampleGripper.getPosition());

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
