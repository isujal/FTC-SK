package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoSequences.PreSpeciDropSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SpeciDropSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SpeciFinalInitSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SpeciInitSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SpeciPickSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SpeciSampleAfterPickSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SpeciSampleAfterPickTwoSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SpeciSampleDropSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SpeciSamplePickSeq;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

import java.util.Arrays;

@Config
//@Deprecated
@Autonomous(name = "Red Speci 1+3 🔴")
//@Deprecated
public class RedSpeciAuto extends LinearOpMode {
    private RobotHardware robot = RobotHardware.getInstance();
    //Subsystems
    Intake intake;
    Outtake outtake;
    Hang hang;

    //Drive
    private MecanumDrive drive = null;
//    static Vector2d samplePick1 = new Vector2d(27, -42);//26, -43)


    //TODO SPECIMEN PICK
    public static double pickOffset = 4;

    //TODO TESTING PROFILE ACCELERATION
    VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(30.0),
            new AngularVelConstraint(Math.PI / 4)
    ));


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        intake = new Intake(robot);
        outtake = new Outtake(robot);

        Pose2d intialPose = new Pose2d(0, 0, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, intialPose);

//        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
//        drive = new MecanumDrive(hardwareMap, startPose);

        //TODO ===============================================TRAJECTORIES =============================================================

        Action trajectoryActionk = drive.actionBuilder(drive.pose)
                .stopAndAdd(()-> new PreSpeciDropSeq(intake,outtake))

                //TODO : PreLoad Drop Pose
                .strafeToLinearHeading(new Vector2d(-29.5, 5), Math.toRadians(0))
                .stopAndAdd(()-> new SpeciDropSeq(intake,outtake))
                .waitSeconds(0.5)
                .stopAndAdd(()-> new SpeciInitSeq(intake,outtake))
                .strafeToLinearHeading(new Vector2d(-20, 5), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-25, 6), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-18, 35), Math.toRadians(90))
                .stopAndAdd(()-> new SpeciSamplePickSeq(intake,outtake))

                // TODO : Sample 1 Strafing Pose
                .strafeToLinearHeading(new Vector2d(-18, 36), Math.toRadians(115))
                .stopAndAdd(()-> new SpeciSampleAfterPickSeq(intake,outtake))
                .waitSeconds(0.3)

                // TODO : Sample 1 Zone Drop Pose
                .strafeToLinearHeading(new Vector2d(-30, 40), Math.toRadians(45))
                .stopAndAdd(()-> new SpeciSampleDropSeq(intake,outtake))
                .waitSeconds(0.3)
                .stopAndAdd(()-> new SpeciInitSeq(intake,outtake))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(-28, 60), Math.toRadians(180))
                .waitSeconds(0.2)
                .stopAndAdd(()-> new SpeciFinalInitSeq(intake,outtake))
                .waitSeconds(0.1)

                // TODO : Sample 1 Pick Pose
                .strafeToLinearHeading(new Vector2d(-4, 60), Math.toRadians(180))
                .waitSeconds(0.3)
                .stopAndAdd(()-> new SpeciPickSeq(intake,outtake))
                .waitSeconds(0.2)
                .stopAndAdd(()-> new PreSpeciDropSeq(intake,outtake))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-10, 60), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-11, 60), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-11, 30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-10, 30), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-38, 30), Math.toRadians(0))
                .waitSeconds(0.5)
                .stopAndAdd(()-> new SpeciDropSeq(intake,outtake))
                .waitSeconds(0.5)
                .stopAndAdd(()-> new SpeciFinalInitSeq(intake,outtake))
                .strafeToLinearHeading(new Vector2d(-10, 30), Math.toRadians(0))














//                .waitSeconds(0.2)
//                .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
//                .waitSeconds(0.2)
//                .strafeToLinearHeading(new Vector2d(-34, -20), Math.toRadians(0))
//                .waitSeconds(0.2)
//                .strafeToLinearHeading(new Vector2d(-7, -20), Math.toRadians(0))
//                .waitSeconds(0.2)
//                .strafeToLinearHeading(new Vector2d(-34, -20), Math.toRadians(0))





                .build();

        //TODO Sample Pick from Obv Zone


        //TODO Sample Drop from Obv Zone


        //TODO Sample 1 Pick

        //TODO Sample 1 Drop


        //TODO Sample 2 Pick


        //TODO Sample 2 Drop


        //TODO Sample 3 Pick

        //TODO Sample 3 Drop


        //TODO Parking


        if (opModeInInit()) {
//            telemetry.addLine("ROBOT INIT MODE");
            Actions.runBlocking(
                    new SequentialAction(
                            intake.IntakeGripperCommands(Intake.SampleGripperState.INIT),
                            intake.IntakeBucketCommands(Intake.IntakeBucketState.PASS),
                            intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF),
                            outtake.LifterCommands(Outtake.LifterState.INIT),
                            outtake.ShoulderCommands(Outtake.ShoulderState.INIT),
                            intake.ExtensionCommands(Intake.ExtensionState.INIT),
                            outtake.SpecimenGripperCommands(Outtake.SpecimenGripperState.CLOSE)

                    ));

        }


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionk
                ));

        while (opModeIsActive()) {
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
//            telemetry.addData("Navx heading (deg)", TwoDeadWheelLocalizer.robotHeading);
            telemetry.update();
        }


    }
}
