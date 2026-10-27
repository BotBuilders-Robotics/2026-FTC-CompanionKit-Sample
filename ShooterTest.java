/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter-only OpMode: two mechanically linked launcher motors on ONE control loop.
 *
 * Controls (gamepad1):
 *   A          - launcher ON
 *   B          - launcher OFF
 *   D-pad up   - target RPM + step
 *   D-pad down - target RPM - step
 */
@TeleOp(name="Shooter Test", group="Linear OpMode")
public class ShooterTest extends LinearOpMode {

    // ---------------- Launcher tuning ----------------
    // Both motors receive the same power every loop. Do NOT use setVelocity() on
    // them - that starts two hub PID loops that fight through the shared shaft.

    // Encoder ticks per output-shaft revolution (bare goBILDA 6000 RPM / REV HD Hex = 28).
    // If your launcher motors have a gearbox, multiply 28 by the gear ratio.
    static final double LAUNCHER_TICKS_PER_REV = 28.0;

    static final double LAUNCHER_DEFAULT_RPM = 3000.0;
    static final double LAUNCHER_RPM_STEP    = 100.0;
    static final double LAUNCHER_MAX_RPM     = 6000.0;
    static final double LAUNCHER_READY_TOLERANCE_RPM = 100.0;

    // Set false if only ONE encoder cable is plugged in (must be "launcher_left").
    static final boolean USE_BOTH_ENCODERS = true;

    // Gains (motor power per encoder tick/second).
    // Tuning: KP = 0, adjust KV until actual ~ target at a few speeds,
    // then raise KP until it recovers quickly without oscillating.
    static final double LAUNCHER_KV = 1.0 / 2800.0;   // feedforward (6000 RPM * 28 / 60 = 2800 tps)
    static final double LAUNCHER_KS = 0.0;            // static friction offset
    static final double LAUNCHER_KP = 0.0005;         // proportional feedback

    static final double NOMINAL_VOLTAGE = 12.0;       // voltage the gains are tuned at
    // -------------------------------------------------

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx launcherLeft = null;
    private DcMotorEx launcherRight = null;
    private VoltageSensor battery = null;

    @Override
    public void runOpMode() {
        launcherLeft  = hardwareMap.get(DcMotorEx.class, "launcher_left");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcher_right");
        battery       = hardwareMap.voltageSensor.iterator().next();

        // Set these so POSITIVE power spins both motors the shooting direction.
        // CHECK BEFORE LINKING THEM - opposing motors will stall and overheat.
        // Reversing a motor also flips its encoder, so both should read positive.
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{launcherLeft, launcherRight}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // our loop, not the hub's
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setPower(0);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double targetRpm = LAUNCHER_DEFAULT_RPM;
        boolean launcherOn = false;
        boolean prevDpadUp = false;
        boolean prevDpadDown = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ---------- Controls ----------
            if (gamepad1.a) launcherOn = true;
            if (gamepad1.b) launcherOn = false;

            if (gamepad1.dpad_up && !prevDpadUp) {
                targetRpm = Math.min(targetRpm + LAUNCHER_RPM_STEP, LAUNCHER_MAX_RPM);
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                targetRpm = Math.max(targetRpm - LAUNCHER_RPM_STEP, 0);
            }
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;

            // ---------- Control loop (one loop, two motors) ----------
            double targetTps   = rpmToTicksPerSec(targetRpm);
            double measuredTps = measureLauncherTps();
            double voltage     = battery.getVoltage();

            double power = 0.0;
            if (launcherOn) {
                double ff = LAUNCHER_KV * targetTps + LAUNCHER_KS;
                double fb = LAUNCHER_KP * (targetTps - measuredTps);
                double voltageScale = (voltage > 1.0) ? NOMINAL_VOLTAGE / voltage : 1.0;
                power = Range.clip((ff + fb) * voltageScale, 0.0, 1.0); // never brake, just coast
            }
            launcherLeft.setPower(power);
            launcherRight.setPower(power);

            double actualRpm = ticksPerSecToRpm(measuredTps);
            boolean ready = launcherOn
                    && Math.abs(actualRpm - targetRpm) < LAUNCHER_READY_TOLERANCE_RPM;

            // ---------- Telemetry ----------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Launcher", launcherOn ? "ON" : "OFF");
            telemetry.addData("RPM", "target %.0f | actual %.0f", targetRpm, actualRpm);
            telemetry.addData("Enc RPM", "left %.0f | right %.0f",
                    ticksPerSecToRpm(launcherLeft.getVelocity()),
                    ticksPerSecToRpm(launcherRight.getVelocity()));
            telemetry.addData("Power", "%.2f  (battery %.1f V)", power, voltage);
            telemetry.addData("Ready", ready ? "YES" : "no");
            telemetry.update();
        }

        launcherLeft.setPower(0);
        launcherRight.setPower(0);
    }

    /** Launcher speed in encoder ticks/second, from one or both encoders. */
    private double measureLauncherTps() {
        if (USE_BOTH_ENCODERS) {
            return (launcherLeft.getVelocity() + launcherRight.getVelocity()) / 2.0;
        }
        return launcherLeft.getVelocity();
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * LAUNCHER_TICKS_PER_REV / 60.0;
    }

    private static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / LAUNCHER_TICKS_PER_REV;
    }
}
