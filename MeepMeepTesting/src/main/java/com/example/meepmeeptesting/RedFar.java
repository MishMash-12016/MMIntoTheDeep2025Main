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
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder (new Pose2d(5.5, -65.5, Math.toRadians(90.00)))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-5, -27, Math.toRadians(90)), Math.toRadians(90)) //preload
                        .setTangent(Math.toRadians(270))
                        .splineToSplineHeading(new Pose2d(-5,-45, Math.toRadians(90)), Math.toRadians(270))

                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(38, -44.84, Math.toRadians(90)), Math.toRadians(0))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(38,-14, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(46,-14 , Math.toRadians(90)), Math.toRadians(0))
                        .setTangent(Math.toRadians(270))
                        .splineToSplineHeading(new Pose2d(46,-57,Math.toRadians(90)), Math.toRadians(270))
                        .setTangent(Math.toRadians(90))
                       .splineToSplineHeading(new Pose2d(46,-14,Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(55,-14 ,Math.toRadians(90)), Math.toRadians(0))
                        .setTangent(Math.toRadians(270))
                        .splineToSplineHeading(new Pose2d(55,-57 ,Math.toRadians(90)), Math.toRadians(270))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(55,-14  ,Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(64,-14  ,Math.toRadians(90)), Math.toRadians(0))
                        .setTangent(Math.toRadians(270))  //pick up first
                        .splineToSplineHeading(new Pose2d(64, -67, Math.toRadians(90)), Math.toRadians(270))
//
                        .setTangent(160)
                        .splineToSplineHeading(new Pose2d(-5, -28, Math.toRadians(90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-5, -45, Math.toRadians(90)), Math.toRadians(90))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


