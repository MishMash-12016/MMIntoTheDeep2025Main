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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.59)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(16.49, 64.22, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(57.52, 60.45, Math.toRadians(-140.00)), Math.toRadians(-54.61))
                                .setTangent(Math.toRadians(240))
                .splineToSplineHeading(new Pose2d(59.34, 36.96, Math.toRadians(270.00)), Math.toRadians(270.00))
                .splineToSplineHeading(new Pose2d(57.52, 60.45, Math.toRadians(-145.00)), Math.toRadians(117.44))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}