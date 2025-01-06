package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Test {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,14.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder (new Pose2d(-14.54, -59.87, Math.toRadians(90.00)))
                        .setTangent(Math.toRadians(130))
                .splineToLinearHeading(new Pose2d(-57.12, -60.32, Math.toRadians(45.00)), Math.toRadians(180.62))
                        .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(-48.84, -44.84, Math.toRadians(60.80)), Math.toRadians(60.80))
                        .setTangent(Math.toRadians(240))
                .splineToLinearHeading(new Pose2d(-57.12, -60.32, Math.toRadians(45.00)), Math.toRadians(270))
                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}