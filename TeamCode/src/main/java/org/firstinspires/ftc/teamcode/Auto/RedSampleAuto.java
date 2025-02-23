package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoSequences.InitSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SampleDropLastSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SampleIntakeAgainSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SampleIntakeLastSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SampleIntakeSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.SampleTransferSeq;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.AutoSequences.SampleDropSeq;
import org.firstinspires.ftc.teamcode.AutoSequences.PreSampleDropSeq;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

import java.util.Arrays;

@Config
//@Deprecated
@Autonomous(name = "Red Sample Safe 🔴")
//@Deprecated
public class RedSampleAuto extends LinearOpMode {
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
    public static Vector2d PreloadDrop = new Vector2d(-57, -55);
    public static Vector2d samplePick1 = new Vector2d(-52, -46.5);

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

        Pose2d intialPose = new Pose2d(-32, -64, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, intialPose);

//        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
//        drive = new MecanumDrive(hardwareMap, startPose);

        //TODO ===============================================TRAJECTORIES =============================================================

        Action trajectoryActionk = drive.actionBuilder(drive.pose)
                .stopAndAdd(() -> new PreSampleDropSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-41, -87), Math.toRadians(45))
                .waitSeconds(0.2)
                .afterTime(0.2, () -> new SampleDropSeq(intake, outtake))
                .waitSeconds(0.3)
                //TODO - 1st Sample Picking
                .strafeToLinearHeading(new Vector2d(-48, -80), Math.toRadians(90))
                .waitSeconds(0.1)
                .stopAndAdd(() -> new SampleIntakeSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-48, -70), Math.toRadians(90))
                .stopAndAdd(() -> new SampleIntakeAgainSeq(intake, outtake))
                .waitSeconds(0.2)
                .stopAndAdd(() -> new SampleTransferSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-49, -83), Math.toRadians(45))
                .stopAndAdd(() -> new PreSampleDropSeq(intake, outtake))
                .waitSeconds(0.2)
                .stopAndAdd(() -> new SampleDropSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-48, -80), Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-46, -80), Math.toRadians(90))
                .waitSeconds(0.2)
                .stopAndAdd(() -> new SampleIntakeSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-46, -70), Math.toRadians(90))
                .stopAndAdd(() -> new SampleIntakeAgainSeq(intake, outtake))
                .waitSeconds(0.2)
                .stopAndAdd(() -> new SampleTransferSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-62, -75), Math.toRadians(45))
                .stopAndAdd(() -> new PreSampleDropSeq(intake, outtake))
                .waitSeconds(0.3)
                .stopAndAdd(() -> new SampleDropLastSeq(intake, outtake))
                .waitSeconds(0.2)
//                .stopAndAdd(() -> new InitSeq(intake, outtake))
// -65,-50   -45,-70
                .strafeToLinearHeading(new Vector2d(-65, -50), Math.toRadians(135))
                .waitSeconds(0.3)
                .stopAndAdd(() -> new SampleIntakeLastSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-68, -47), Math.toRadians(135))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-65, -50), Math.toRadians(135))
                .waitSeconds(0.2)
                .stopAndAdd(() -> new SampleIntakeAgainSeq(intake, outtake))
                .waitSeconds(0.2)
                .stopAndAdd(() -> new SampleTransferSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-62, -75), Math.toRadians(45))
                .stopAndAdd(() -> new PreSampleDropSeq(intake, outtake))
                .waitSeconds(0.3)
                .afterTime(0.2, () -> new SampleDropLastSeq(intake, outtake))
                .waitSeconds(0.1)
                //TODO - 1st Sample Picking
                .strafeToLinearHeading(new Vector2d(-48, -80), Math.toRadians(90))

                .build();


        TrajectoryActionBuilder trajectoryAction0 = drive.actionBuilder(drive.pose)
                //TODO - Preload Sample Droping
                .stopAndAdd(() -> new PreSampleDropSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-43, -87), Math.toRadians(45))
                .waitSeconds(0.5)
                .afterTime(0.2, () -> new SampleDropSeq(intake, outtake))
                .waitSeconds(0.3)
                //TODO - 1st Sample Picking
                .strafeToLinearHeading(new Vector2d(-46, -80), Math.toRadians(90))
                .waitSeconds(0.4)
                .stopAndAdd(() -> new SampleIntakeSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-46, -70), Math.toRadians(90))
                .stopAndAdd(() -> new SampleIntakeAgainSeq(intake, outtake))
                .waitSeconds(0.4)
                .stopAndAdd(() -> new SampleTransferSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-49, -83), Math.toRadians(45))
                .stopAndAdd(() -> new PreSampleDropSeq(intake, outtake))
                .waitSeconds(0.4)
                .stopAndAdd(() -> new SampleDropSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-46, -80), Math.toRadians(90))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(-44, -80), Math.toRadians(90))
                .waitSeconds(0.5)
                .stopAndAdd(() -> new SampleIntakeSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-44, -70), Math.toRadians(90))
                .stopAndAdd(() -> new SampleIntakeAgainSeq(intake, outtake))
                .waitSeconds(0.5)
                .stopAndAdd(() -> new SampleTransferSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-62, -75), Math.toRadians(45))
                .stopAndAdd(() -> new PreSampleDropSeq(intake, outtake))
                .waitSeconds(0.5)
                .stopAndAdd(() -> new SampleDropLastSeq(intake, outtake))
                .waitSeconds(0.5)
                .stopAndAdd(() -> new InitSeq(intake, outtake))
                .strafeToLinearHeading(new Vector2d(-62, -70), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-62, -30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-61, -30), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-35, -30), Math.toRadians(0));




// -65,-50   -45,-70




        TrajectoryActionBuilder trajectoryAction = drive.actionBuilder(intialPose)
                //TODO Preload Sample Drop

//                .stopAndAdd(() -> new PreSampleDropSeq(intake, outtake))
                //predrop
                .strafeToLinearHeading(new Vector2d(-65, -54.5), Math.toRadians(75))
//                .waitSeconds(0.5)
//                .stopAndAdd(()-> new SampleDropSeq(intake,outtake))
//                .waitSeconds(1)
                .strafeToLinearHeading(samplePick1, Math.toRadians(90));
//                .waitSeconds(0.5)
//                .stopAndAdd(()-> new SampleTransferSeq(intake, outtake));
////                .waitSeconds(0.5)
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-35,-85), Math.toRadians(45))
//
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-40,-90), Math.toRadians(45))
//                //drop
////                .waitSeconds(0.5)
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-27,-90), Math.toRadians(90))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-27,-85), Math.toRadians(90))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-35,-85), Math.toRadians(45))
//
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-40,-90), Math.toRadians(45))
//                //drop
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-35,-85), Math.toRadians(45))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-28,-56), Math.toRadians(135))
//                .waitSeconds(0.5)
////                .strafeToLinearHeading(new Vector2d(-33,-82), Math.toRadians(135))
////                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-35,-85), Math.toRadians(45))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-40,-90), Math.toRadians(45))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(-35,-85), Math.toRadians(45))
//
//
//
//
////                .stopAndAdd(() -> new SampleDropSeq(intake, outtake))
//                .waitSeconds(1);
////                .stopAndAdd(()->new AfterDropSeq(intake,outtake));

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
                            intake.ExtensionCommands(Intake.ExtensionState.INIT)
                    ));

        }


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction0.build()
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
