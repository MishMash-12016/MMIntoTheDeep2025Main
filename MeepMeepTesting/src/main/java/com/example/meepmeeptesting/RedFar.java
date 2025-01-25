package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedFar {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,14.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder (new Pose2d(5.5, -65.5, Math.toRadians(90.00)))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-5, -28, Math.toRadians(90)), Math.toRadians(90))
                        .lineTo(new Vector2d(-5,-45))

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(40, -44.84, Math.toRadians(90)), Math.toRadians(0))
                        .strafeTo(new Vector2d(40,-15))
                        .strafeTo(new Vector2d(46,-15))

                        .strafeTo(new Vector2d(45,-54)) //- wait 200 sec
                        .lineTo(new Vector2d(45,-60))

                        .setTangent(Math.toRadians(120))
                        .splineToLinearHeading(new Pose2d(-3, -25, Math.toRadians(90)), Math.toRadians(90))
                        .lineTo(new Vector2d(-3,-50))

                        .setTangent(Math.toRadians(0)) //pick up first
                        .splineToLinearHeading(new Pose2d(45, -54, Math.toRadians(90)), Math.toRadians(0))
                        .lineTo(new Vector2d(45,-59))

//                        .setTangent(Math.toRadians(90))
//                        .splineToSplineHeading(new Pose2d(-3, -28, Math.toRadians(90)), Math.toRadians(90))
//                        .lineTo(new Vector2d(-3,-50))
//
//                        .setTangent(Math.toRadians(0))
//                        .splineToLinearHeading(new Pose2d(45, -59, Math.toRadians(90)), Math.toRadians(0))



                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


