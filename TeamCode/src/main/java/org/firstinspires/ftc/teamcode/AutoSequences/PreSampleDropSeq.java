package org.firstinspires.ftc.teamcode.AutoSequences;//package org.firstinspires.ftc.teamcode.sequences;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.instantcommands.IntakeGripperCommand;
import org.firstinspires.ftc.teamcode.instantcommands.IntakeRollerCommand;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

public class PreSampleDropSeq {
        public PreSampleDropSeq(Intake intake, Outtake outtake) {
                Actions.runBlocking(
                        new SequentialAction(
                                intake.IntakeRollerCommands(Intake.IntakeRollerState.ON),
                                outtake.ShoulderCommands(Outtake.ShoulderState.PASS),
                                new SleepAction(0.1),
                                new ParallelAction(
                                        intake.IntakeRollerCommands(Intake.IntakeRollerState.ON),
                                        intake.IntakeGripperCommands(Intake.SampleGripperState.CLOSE)
                                        ),
                                new SleepAction(0.1),
                                outtake.LifterCommands(Outtake.LifterState.HIGH),
                                new SleepAction(1),
                                outtake.ShoulderCommands(Outtake.ShoulderState.DROP)
                        )                                );
        }

}


