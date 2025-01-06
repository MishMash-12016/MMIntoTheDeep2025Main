package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Test {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder (new Pose2d(-24.38, -67.88, Math.toRadians(90.00)))
                        .splineToLinearHeading(new Pose2d(-60.10, -59.41, Math.toRadians(164.20)), Math.toRadians(164.20))
                        .splineToLinearHeading(new Pose2d(-48.88, -31.02, Math.toRadians(69.13)), Math.toRadians(69.13))
                        .splineToLinearHeading(new Pose2d(-50.25, -33.31, Math.toRadians(250.77)), Math.toRadians(250.77))
                        .splineToLinearHeading(new Pose2d(-61.70, -61.47, Math.toRadians(247.88)), Math.toRadians(247.88))
                        .splineToLinearHeading(new Pose2d(-59.64, -31.02, Math.toRadians(86.13)), Math.toRadians(86.13))
                        .splineToLinearHeading(new Pose2d(-65.36, -58.03, Math.toRadians(270.00)), Math.toRadians(270.00))
                        .splineToLinearHeading(new Pose2d(-69.94, -30.10, Math.toRadians(270.00)), Math.toRadians(270.00))
                        .splineToLinearHeading(new Pose2d(-63.07, -55.75, Math.toRadians(-75.00)), Math.toRadians(-75.00))
                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}