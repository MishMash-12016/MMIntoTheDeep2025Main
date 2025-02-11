package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class EyalsSampleAutonomous {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(11.417,16.535)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(720), Math.toRadians(720), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-39, -62.73, Math.toRadians(180)))

                        .lineToLinearHeading(new Pose2d(-50, -62.73, Math.toRadians(180)))

                        .lineToSplineHeading(new Pose2d(-58.49, -47.7, Math.toRadians(246)))

                        .lineToLinearHeading(new Pose2d(-63.4, -49.3, Math.toRadians(258.9)))

                        .lineToLinearHeading(new Pose2d(-50.8, -45, Math.toRadians(315)))

                        .lineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)))

                        .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


