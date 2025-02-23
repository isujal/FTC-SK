package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class SampleDrop {
    public static Action PreDrop(Intake intake, Outtake outtake) {
        return new SequentialAction(
//                new ParallelAction(
                        outtake.LifterCommands(Outtake.LifterState.HIGH),
                        new SleepAction(1),
                        outtake.ShoulderCommands(Outtake.ShoulderState.DROP)//PRE_DROP
//                )
        );
    }

    public static Action Drop(Intake intake, Outtake outtake) {
        return new SequentialAction(
                new ParallelAction(
                        outtake.ShoulderCommands(Outtake.ShoulderState.DROP),
                        intake.IntakeBucketCommands(Intake.IntakeBucketState.TRANSFER)
                ),
                new SleepAction(0.5),
                intake.IntakeGripperCommands(Intake.SampleGripperState.OPEN),
                new SleepAction(1),
                intake.IntakeGripperCommands(Intake.SampleGripperState.INIT),
                intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF),
                outtake.LifterCommands(Outtake.LifterState.INIT),
                outtake.ShoulderCommands(Outtake.ShoulderState.INIT),
                intake.ExtensionCommands(Intake.ExtensionState.INIT),
                new SleepAction(2),
                intake.IntakeBucketCommands(Intake.IntakeBucketState.PASS)
        );
    }

    public static Action Release(Intake intake, Outtake outtake) {
        return new SequentialAction(
                intake.IntakeRollerCommands(Intake.IntakeRollerState.RELEASE)
        );
    }

    public static Action OFF(Intake intake, Outtake outtake) {
        return new SequentialAction(
                intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF)
        );
    }
    public static Action AfterDrop(Intake intake, Outtake outtake) {
        return new SequentialAction(
                outtake.ShoulderCommands(Outtake.ShoulderState.DROP),
                intake.IntakeGripperCommands(Intake.SampleGripperState.OPEN),
                new SleepAction(0.5),
                outtake.ShoulderCommands(Outtake.ShoulderState.INIT)
        );
    }




}
