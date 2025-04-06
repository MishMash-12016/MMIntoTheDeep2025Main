package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class oneplusfive {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

         double maxWheelVel = 70;
        double maxProfileAccel = 70;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,40)//14.5
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 70, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(5.5, -62.73, Math.toRadians(90.00)))

//                        TrajectoryActionBuilder driveToScorePreload = drive.actionBuilder(currentPose)
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(5.5, -28), Math.toRadians(90))
//                                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
//                                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.4));


        //there isn't driveToEject its all conspiracy
//        TrajectoryActionBuilder driveToEject = drive.actionBuilder(new Pose2d(5.5, -29, Math.toRadians(270)))
                .setTangent(Math.toRadians(260))
                .splineToLinearHeading(new Pose2d(25, -45, Math.toRadians(140)), Math.toRadians(0))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));


//        TrajectoryActionBuilder driveToPush1 = driveToEject.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(28, -37, Math.toRadians(215)), Math.toRadians(50))

                        .turn(Math.toRadians(-85))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


