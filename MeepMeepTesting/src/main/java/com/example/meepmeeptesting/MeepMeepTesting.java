package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions( 44 / 2.54,39 / 2.54)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-10.79, -62.61, Math.toRadians(90.00)))
                                .splineToLinearHeading(new Pose2d(-54.24, -55.85, Math.toRadians(45.00)), Math.toRadians(175.31))
                                .splineToLinearHeading(new Pose2d(-48.51, -44.55, Math.toRadians(90.00)), Math.toRadians(65.00))
                                .setTangent(Math.toRadians(240))
                                .splineToLinearHeading(new Pose2d(-54.24, -55.41, Math.toRadians(45.00)), Math.toRadians(245))
                                .build()
);


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}