package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TwoRobotsTesting {

    //collection from human player
    private static final Pose2d collectionSpecimanPos = new Pose2d(45,-67,Math.toRadians(90));


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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(5.5, -65.5, Math.toRadians(90.00)))

                        //score 1
                        .splineToConstantHeading(scoreSpecimanPos,Math.toRadians(90))
                        .splineToConstantHeading(scoreSpecimanPos2,Math.toRadians(270))

//                        //push 1
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(pushSamplePos, Math.toRadians(0))
                        .addTemporalMarkerOffset(1, () -> {
                            // I don't have access to MMRobot, just add it to the imports and write the commands
                            //open linear - max opening
                            //intake arm - might need a new position
                            //intake rotator - turn 90 degrees

                            //you might need to close the liner intake after each "push"
                        })
                        .setTangent(Math.toRadians(300))
                        .splineToLinearHeading(new Pose2d(32.24,-53, Math.toRadians(120)),Math.toRadians(240))

                        .setTangent(Math.toRadians(80))
                        .splineToLinearHeading(new Pose2d(33.24+6,-38,Math.toRadians(230)),Math.toRadians(70))
                        .setTangent(Math.toRadians(300))
                        .splineToLinearHeading(new Pose2d(32.25+6,-53, Math.toRadians(120)),Math.toRadians(240))
//                        //push 2
//                        .splineToLinearHeading(pushSamplePos.plus(new Pose2d(pushingSampleXConst,0,0)),Math.toRadians(180))
//                        .addTemporalMarkerOffset(1, () -> {
//
//
//                        })
//                        .turn(Math.toRadians(-90))
//                        //push 3
//                        .splineToLinearHeading(pushSamplePos.plus(new Pose2d(pushingSampleXConst*2,0,0)),Math.toRadians(180))
//                        .addTemporalMarkerOffset(1, () -> {
//
//
//                        })
//                        .turn(Math.toRadians(-70))
//
//
//
//                        //score 2
//                        .addTemporalMarkerOffset(0, () -> {
//                            // I don't have access to MMRobot, just add it to the imports and write the commands
//                            //for prepare speciman collection
//                        })
//                        .splineToLinearHeading(collectionSpecimanPos,Math.toRadians(250)) //a change here might be needed, change angle to 270
//                        .addTemporalMarkerOffset(0, () -> {
//                            // I don't have access to MMRobot, just add it to the imports and write the commands
//                            //for speciman collection
//                        })
//                        .splineToConstantHeading(scoreSpecimanPos.minus(new Vector2d(scoreSpecimanXConst,0)), Math.toRadians(90))
//                        .splineToConstantHeading(scoreSpecimanPos2.minus(new Vector2d(scoreSpecimanXConst,0)), Math.toRadians(90))
//
//                        //score 3
//                        .addTemporalMarkerOffset(0, () -> {
//                            // I don't have access to MMRobot, just add it to the imports and write the commands
//                            //for prepare speciman collection
//                        })
//                        .splineToLinearHeading(collectionSpecimanPos,Math.toRadians(270))
//                        .waitSeconds(.000001) //this time delay is for adjusting the path without splitting it in to two paths
//                        .addTemporalMarkerOffset(0, () -> {
//                            // I don't have access to MMRobot, just add it to the imports and write the commands
//                            //for speciman collection
//                        })
//                        .splineToConstantHeading(scoreSpecimanPos.minus(new Vector2d(scoreSpecimanXConst*2,0)), Math.toRadians(90))
//                        .splineToConstantHeading(scoreSpecimanPos2.minus(new Vector2d(scoreSpecimanXConst*2,0)),Math.toRadians(90))
//
//                        //score 4
//                        .addTemporalMarkerOffset(0, () -> {
//                            // I don't have access to MMRobot, just add it to the imports and write the commands
//                            //for prepare speciman collection
//                        })
//                        .splineToLinearHeading(collectionSpecimanPos,Math.toRadians(270))
//                        .waitSeconds(.000001)
//                        .addTemporalMarkerOffset(0, () -> {
//                            // I don't have access to MMRobot, just add it to the imports and write the commands
//                            //for speciman collection
//                        })
//                        .splineToConstantHeading(scoreSpecimanPos.minus(new Vector2d(scoreSpecimanXConst*3,0)), Math.toRadians(90))
//                        .splineToConstantHeading(scoreSpecimanPos2.minus(new Vector2d(scoreSpecimanXConst*3,0)),Math.toRadians(90))
//
//                        //score 5
//                        .addTemporalMarkerOffset(0, () -> {
//                            // I don't have access to MMRobot, just add it to the imports and write the commands
//                            //for prepare speciman collection
//                        })
//                        .splineToLinearHeading(collectionSpecimanPos,Math.toRadians(270))
//                        .waitSeconds(.000001)
//                        .addTemporalMarkerOffset(0, () -> {
//                            // I don't have access to MMRobot, just add it to the imports and write the commands
//                            //for speciman collection
//                        })
//                        .splineToConstantHeading(scoreSpecimanPos.minus(new Vector2d(scoreSpecimanXConst*4,0)), Math.toRadians(90))
//                        .strafeTo(scoreSpecimanPos2.minus(new Vector2d(scoreSpecimanXConst*4)))
//
//                        //park
//                        .splineToLinearHeading(parkPos, Math.toRadians(0))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}