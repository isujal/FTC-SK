package org.firstinspires.ftc.teamcode.Sequence;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class SampleTransferSeq {
    public static Action TransferSeq(Intake intake, Outtake outtake) {
        return new SequentialAction(
                intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF),
                outtake.LifterCommands(Outtake.LifterState.INIT),
                new ParallelAction(
                        intake.IntakeGripperCommands(Intake.SampleGripperState.OPEN),
                        intake.IntakeBucketCommands(Intake.IntakeBucketState.PASS)
                ),
                intake.ExtensionCommands(Intake.ExtensionState.HALF),
                new SleepAction(1),
                outtake.ShoulderCommands(Outtake.ShoulderState.PASS),
                new SleepAction(0.5),
                intake.IntakeGripperCommands(Intake.SampleGripperState.CLOSE)
        );
    }

}
