package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TwoRobotsTesting {

    //collection from human player
    private static final Pose2d collectionSpecimanPos = new Pose2d(42,-67,Math.toRadians(90));


    //scoring positions
    private static final Vector2d scoreSpecimanPos = new Vector2d(5.5, -27); // when driving to scoring location
    private static final Vector2d scoreSpecimanPos2 = new Vector2d(5.5, -40); // when driving backwards to score
    private static final int scoreSpecimanXConst = 3; // the spacing of the scored specimens


    //pushing to human player
    private static final Pose2d pushSamplePos = new Pose2d(33.24,-38, Math.toRadians(230));
    private static final float pushingSampleXConst = 10; // the spacing between the samples
    /*
     * a adjustive const for each sample might be needed
     */

    //parking position
    private static final Pose2d parkPos = new Pose2d(45, -60, Math.toRadians(90));


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(11.02,14.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(720), Math.toRadians(720), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(5.5, 65.5, Math.toRadians(270)))

                        .splineToConstantHeading(new Vector2d(5.5, 0), Math.toRadians(90))


                        //.lineTo(new Vector2d(0, -50))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}