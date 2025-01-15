package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TestRedClose {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,14.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(-5.5, -65.5, Math.toRadians(90.00)))
                        .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-5, -30, Math.toRadians(90)), Math.toRadians(90)) //score secimen
                        .setTangent(Math.toRadians(240))
                .splineToLinearHeading(new Pose2d(-48.84, -44.84, Math.toRadians(270)), Math.toRadians(180)) //pick up 1
                        .setTangent(Math.toRadians(240))
                .splineToLinearHeading(new Pose2d(-55.97, -59.18, Math.toRadians(225)), Math.toRadians(250)) //go back
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-57.875, -44.84, Math.toRadians(270)), Math.toRadians(95)) //pick up 2
                .setTangent(Math.toRadians(275))
                .splineToLinearHeading(new Pose2d(-55.97, -59.18, Math.toRadians(225)), Math.toRadians(270)) //go back
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-50.8, -45.8, Math.toRadians(315)), Math.toRadians(95)) //pick up 3
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-55.97, -59.18, Math.toRadians(225)), Math.toRadians(270)) //go back

                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}