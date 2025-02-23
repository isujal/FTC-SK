package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class SplineSpecimen {
    static Vector2d samplePick1 = new Vector2d(26, -43);
    public static Vector2d specimenPick1 = new Vector2d(29.5, -64.5);
    public static Vector2d specimenPick2 = new Vector2d(27 + 5, -66);
    public static Vector2d specimenPick3 = new Vector2d(27 + 5, -66);



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set Dimensions
                .setDimensions(11.5, 16)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(90.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(17.5, -64, Math.toRadians(90)))
//                .strafeToLinearHeading(new Vector2d(2,-30), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(2, -30.5), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(2, -38.5), Math.toRadians(90))
                .setReversed(true)
                //sample 1
                .splineToLinearHeading(new Pose2d(27,-40 , Math.toRadians(30)),Math.toRadians(30)) // x - 42

//                .strafeToLinearHeading(new Vector2d(27,-40), Math.toRadians(30))
                // TODO - PRELOAD
//                .strafeToConstantHeading((new Vector2d(2, -30.5)))
//                .strafeToConstantHeading((new Vector2d(2, -38)))
//                .strafeToLinearHeading(samplePick1, Math.toRadians(30))
//                .strafeToLinearHeading(new Vector2d(35, -49), Math.toRadians(-30))//34, -49
//                .strafeToLinearHeading(new Vector2d(35, -38), Math.toRadians(30))//new Vector2d(34, -40)
//                .strafeToLinearHeading(new Vector2d(40, -50), Math.toRadians(-30))
//                .strafeToLinearHeading(new Vector2d(46, -38), Math.toRadians(30))//new Vector2d(35, -38)
//                .strafeToLinearHeading(new Vector2d(34, -50), Math.toRadians(-30))
//
//                .strafeToLinearHeading(new Vector2d(30.5, -63), Math.toRadians(90))//new Vector2d(43, -45.01)
//                //TODO PICK SPECIEMEN 1
//                .strafeToLinearHeading(specimenPick1, Math.toRadians(90))//new Vecto
//                .strafeToLinearHeading(new Vector2d(0, -30.5), Math.toRadians(90))
//                .strafeToLinearHeading(specimenPick2, Math.toRadians(90))//new Vector2d(27, -64.5)
//                .strafeToLinearHeading(new Vector2d(2, -30.5), Math.toRadians(90))
//                .strafeToLinearHeading(specimenPick3, Math.toRadians(90))//new Vector2d(27, -64.5)
//                .strafeToLinearHeading(new Vector2d(4, -30.5), Math.toRadians(90))
//                .strafeToLinearHeading(specimenPick3, Math.toRadians(90))//new Vector2d(27, -64.5)
//                .strafeToLinearHeading(new Vector2d(4, -30.5), Math.toRadians(90))
//                .strafeToLinearHeading(specimenPick3, Math.toRadians(90))//new Vector2d(27, -64.5)
//                .strafeToLinearHeading(new Vector2d(4, -30.5), Math.toRadians(90))

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

/*
Regular one take 18.76
Alliance friendly takes 19.95  but this one will works like a delay
TODO ALLIANCE FRIENDLY

  .strafeToLinearHeading(samplePick1, Math.toRadians(30))
                .strafeToLinearHeading(new Vector2d(35, -49), Math.toRadians(-30))//34, -49
                .strafeToLinearHeading(new Vector2d(35, -38), Math.toRadians(30))//new Vector2d(34, -40)
                .strafeToLinearHeading(new Vector2d(40, -50), Math.toRadians(-30))
                .strafeToLinearHeading(new Vector2d(46, -38), Math.toRadians(30))//new Vector2d(35, -38)
                .strafeToLinearHeading(new Vector2d(34, -50), Math.toRadians(-30))

                .strafeToLinearHeading(new Vector2d(30.5, -63), Math.toRadians(90))//new Vector2d(43, -45.01)
                //TODO PICK SPECIEMEN 1
                .strafeToLinearHeading(specimenPick1, Math.toRadians(90))//new Vecto
                .strafeToLinearHeading(new Vector2d(0, -30.5), Math.toRadians(90))
                .strafeToLinearHeading(specimenPick2, Math.toRadians(90))//new Vector2d(27, -64.5)
                .strafeToLinearHeading(new Vector2d(2, -30.5), Math.toRadians(90))
                .strafeToLinearHeading(specimenPick3, Math.toRadians(90))//new Vector2d(27, -64.5)
                .strafeToLinearHeading(new Vector2d(4, -30.5), Math.toRadians(90))
                .strafeToLinearHeading(specimenPick3, Math.toRadians(90))//new Vector2d(27, -64.5)
                .strafeToLinearHeading(new Vector2d(4, -30.5), Math.toRadians(90))
                .strafeToLinearHeading(specimenPick3, Math.toRadians(90))//new Vector2d(27, -64.5)
                .strafeToLinearHeading(new Vector2d(4, -30.5), Math.toRadians(90))
 */