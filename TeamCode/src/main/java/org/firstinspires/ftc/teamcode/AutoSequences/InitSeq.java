package org.firstinspires.ftc.teamcode.AutoSequences;//package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class InitSeq {
    public InitSeq(Intake intake, Outtake outtake) {
        Actions.runBlocking(
                new SequentialAction(
                        intake.IntakeGripperCommands(Intake.SampleGripperState.INIT),
                        intake.IntakeBucketCommands(Intake.IntakeBucketState.TRANSFER),
                        intake.IntakeRollerCommands(Intake.IntakeRollerState.OFF),
                        outtake.LifterCommands(Outtake.LifterState.INIT),
                        outtake.ShoulderCommands(Outtake.ShoulderState.PASS),
                        intake.ExtensionCommands(Intake.ExtensionState.INIT),
                        new SleepAction(1),
                        intake.IntakeBucketCommands(Intake.IntakeBucketState.PASS)

                        )                          );
    }

}










