package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(6, -60, Math.toRadians(90)))

                // TODO - PRELOAD
                .splineToConstantHeading(new Vector2d(2, -27.5), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(2, -40), Math.toRadians(90))
                .setReversed(true)

                // TODO - Sliding 1st Sample

                .strafeToLinearHeading(new Vector2d(27,-40), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(47.5,-14 , Math.toRadians(90)),Math.toRadians(-40)) // x - 42
                .strafeToLinearHeading(new Vector2d(48,-42.5), Math.toRadians(90))

                // TODO - Sliding 2st Sample
                .strafeToLinearHeading(new Vector2d(45,-40), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(56,-14, Math.toRadians(90)),Math.toRadians(-40))
                .strafeToLinearHeading(new Vector2d(56,-42.5), Math.toRadians(90))
                .waitSeconds(1)

                // TODO - Pick up 1st Specimen
//                .strafeToLinearHeading(new Vector2d(36,-55), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(36,-62), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(28,-62,Math.toRadians(90)),Math.toRadians(-90))

                // TODO - Droping 1st Specimen
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(0,-28.5, Math.toRadians(90)),Math.toRadians(90))
//                                .strafeTo(new Vector2d(0,-28.5))

                //TODO -  Pick up 2nd Specimen
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(33,-62, Math.toRadians(90)),Math.toRadians(-45))

                // TODO - Droping 2nd Specimen
                .splineToLinearHeading(new Pose2d(-1.5,-27, Math.toRadians(90)),Math.toRadians(90))

                //TODO - Picking 3rd Specimen
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36,-62, Math.toRadians(90)),Math.toRadians(-45))

                //TODO - Droping 3rd Specimen
                .splineToLinearHeading(new Pose2d(0,-27, Math.toRadians(90)),Math.toRadians(90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}