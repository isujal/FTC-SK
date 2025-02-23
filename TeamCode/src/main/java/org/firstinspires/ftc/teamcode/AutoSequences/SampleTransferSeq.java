package org.firstinspires.ftc.teamcode.AutoSequences;//package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class SampleTransferSeq {
    public SampleTransferSeq(Intake intake, Outtake outtake) {
        Actions.runBlocking(
                new SequentialAction(

                        intake.IntakeBucketCommands(Intake.IntakeBucketState.PASS),
                        intake.ExtensionCommands(Intake.ExtensionState.HALF),
                        new SleepAction(1),
                        outtake.ShoulderCommands(Outtake.ShoulderState.PASS),
                        new SleepAction(0.5),
                        intake.IntakeGripperCommands(Intake.SampleGripperState.CLOSE)
                ));
    }

}


