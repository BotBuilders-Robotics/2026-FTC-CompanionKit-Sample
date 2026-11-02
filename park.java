package org.firstinspires.ftc.teamcode.tread;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.treadpathing.follower.Follower;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.route.Action;
import org.treadpathing.route.InstantAction;
import org.treadpathing.route.Route;

/**
 * A complete autonomous, showing every primitive the library has.
 *
 * <p>Read the route out loud and it describes what the robot does. That is the whole point of
 * the builder.
 *
 * <h3>Three things to notice</h3>
 *
 * <b>Headings are the robot's headings.</b> Every pose is where the nose points, which is what
 * you see standing at the field. On a reversed segment the library flips the travel direction
 * for you.
 *
 * <p><b>{@code reversed()} inserts a cusp.</b> The path being accumulated is closed off, the
 * robot comes to a stop, and a new path starts the other way. That is the only honest way for
 * a differential drive to change direction, and doing it explicitly beats meeting it as a
 * mystery lurch halfway through a match.
 *
 * <p><b>{@code stopAndHold} is what makes it accurate.</b> Drive segments get the robot
 * roughly where it is going, quickly. The pose hold is what gets it exactly there, and it is
 * where you should spend your route's time budget when position matters.
 */
@Autonomous(name = "Park", group = "tread")
public class Park extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.buildFollower(hardwareMap);

        follower.setPose(new Pose(13.75, 130.75, Math.toRadians(-90.75)));

        Route route = follower.route()
        .splineTo(new Pose(8.25, 115.50, Math.toRadians(112.72)))
        .build();

        telemetry.addLine(route.summary());
        telemetry.addLine();
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        follower.follow(route);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addLine(follower.telemetry());
            telemetry.update();
        }

        follower.breakFollowing();

        while (opModeIsActive()) {
            telemetry.addData("finished at", follower.getPose());
            telemetry.addData("planned", route.getPlannedEnd());
            telemetry.addData("error (in)", "%.2f",
                    follower.getPose().distanceTo(route.getPlannedEnd()));
            telemetry.update();
        }
    }
}
