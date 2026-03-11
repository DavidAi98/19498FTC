package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import java.util.Map;

/**
 * VirtualGoalSolver — Move-While-Shooting compensation.
 *
 * CONCEPT:
 *   When the robot moves, the ball inherits the robot's velocity as it leaves the shooter.
 *   From the ball's perspective the goal appears to be at a different position than where
 *   odometry says it is. We call this shifted position the "virtual goal."
 *   Aiming at the virtual goal instead of the real goal makes the ball land on target
 *   regardless of how the robot is moving.
 *
 * HOW THE ITERATION WORKS:
 *   1. Get shot RPM + hood angle for the current distance estimate.
 *   2. Compute actual ball flight time from those ballistic parameters.
 *   3. Add BALL_TRANSFER_TIME (delay from fire command to ball leaving shooter).
 *   4. Shift the real goal backwards by (turretVelocity × totalTime) to get the virtual goal.
 *      ("Backwards" because the robot keeps moving during that time, so the goal appears to drift.)
 *   5. Recompute the distance to the virtual goal.
 *   6. Repeat 5 times — converges quickly (usually within 2 iterations).
 *
 * TURRET TIP VELOCITY:
 *   The turret is offset from the robot center. When the robot rotates (omega ≠ 0),
 *   the turret tip has an additional tangential velocity even if the robot isn't translating.
 *   This is included in turretVelocity = linearVel + (omega × turretOffset).
 *
 * UNITS: all distances in inches, velocities in inches/sec, angles in degrees for output.
 *
 * TUNING (see Constant.java):
 *   SHOOTER_WHEEL_DIAMETER_INCH — physical flywheel diameter. Affects RPM → ball speed conversion.
 *   BALL_SPEED_EFFICIENCY       — ratio of ball exit speed to flywheel surface speed (typically 0.7–0.9).
 *   BALL_TRANSFER_TIME          — seconds from fire signal to ball leaving shooter (typically 0.05–0.2s).
 */
public class VirtualGoalSolver {

    /**
     * Output of the solver.
     *
     * virtGoalX / virtGoalY:
     *   Field-frame coordinates of the virtual goal (inches).
     *   Pass these to MecanumDrive.angleToPoint() to get the compensated turret angle.
     *
     * effectiveDistInch:
     *   Distance from turret to the virtual goal (inches).
     *   Pass this to shooter.updateShootingParams() instead of the raw odometry distance
     *   so the RPM and hood angle are also compensated.
     *
     * turretOmegaRad:
     *   How fast the turret needs to rotate (rad/s) to track the virtual goal while the
     *   robot is moving. Currently informational — could be used as a servo feedforward
     *   in the future if your turret controller supports velocity feedforward.
     */
    public static class ShotSolution {
        public final double virtGoalX;
        public final double virtGoalY;
        public final double effectiveDistInch;
        public final double turretOmegaRad;
        public final double compScale; // 0 = no comp (close range), 1 = full comp

        public ShotSolution(double vgx, double vgy, double dist, double omega, double scale) {
            this.virtGoalX       = vgx;
            this.virtGoalY       = vgy;
            this.effectiveDistInch = dist;
            this.turretOmegaRad  = omega;
            this.compScale       = scale;
        }
    }

    /**
     * Solve for the virtual goal given the current robot state.
     *
     * @param turretX    Turret position X (inches, field frame) — from MecanumDrive.turretX
     * @param turretY    Turret position Y (inches, field frame) — from MecanumDrive.turretY
     * @param velX       Robot linear velocity X (in/s, field frame) — from MecanumDrive.velX
     * @param velY       Robot linear velocity Y (in/s, field frame) — from MecanumDrive.velY
     * @param omega      Robot angular velocity (rad/s) — from MecanumDrive.omega
     * @param headingRad Robot heading (radians) — from MecanumDrive.headingRad
     * @param goalX      Real goal X (inches, field frame) — Constant.GOAL_CENTER_X
     * @param goalY      Real goal Y (inches, field frame) — Constant.BLUE/RED_GOAL_CENTER_Y
     */
    public static ShotSolution solve(
            double turretX, double turretY,
            double velX,    double velY,
            double omega,   double headingRad,
            double goalX,   double goalY) {


        // Turret offset vector in field frame.
        // In robot frame the turret sits at (-TURRET_OFFSET, 0) — behind center.
        // Rotating to field frame: multiply by rotation matrix.

        double offsetFieldX = -Constant.TURRET_OFFSET * Math.cos(headingRad);
        double offsetFieldY = -Constant.TURRET_OFFSET * Math.sin(headingRad);

        // Total turret tip velocity = robot linear velocity
        //                           + tangential velocity from robot rotation.
        // Tangential vel formula (2D cross product): v = omega × r
        //   v_x = -omega * r_y
        //   v_y =  omega * r_x

        double turretVelX = velX + (-omega * offsetFieldY);
        double turretVelY = velY + ( omega * offsetFieldX);

        double virtGoalX = goalX;
        double virtGoalY = goalY;
        double currentDist = Math.hypot(goalX - turretX, goalY - turretY);

        // Scale compensation down at close range.
        // At short distances the same velocity shift produces a much larger angle change.
        // e.g. 2 inch virtual goal shift at 15 inches = ~7 degrees off — way too much.
        // This ramps from 0 compensation at COMP_MIN_DIST to full at COMP_MAX_DIST.
        // Tune these in Constant.java if turret still drifts when close.
        double rawDist = Math.hypot(goalX - turretX, goalY - turretY);
        double compScale = (rawDist - Constant.COMP_MIN_DIST)
                / (Constant.COMP_MAX_DIST - Constant.COMP_MIN_DIST);
        compScale = Math.max(0.0, Math.min(1.0, compScale));

        // TUNE: LATERAL_COMP_BOOST — start at 1.6, decrease if left becomes too much.
        boolean movingAwayInY = turretVelY * (turretY - goalY) > 0;
        double lateralBoost = movingAwayInY ? Constant.LATERAL_COMP_BOOST : 1.0;

        double scaledVelX = turretVelX * compScale;
        double scaledVelY = turretVelY * compScale * lateralBoost;

        for (int i = 0; i < 5; i++) {
            // Get RPM and hood angle for the current distance estimate
            double[] params = getShotParams(currentDist);  // {RPM, hoodAngleDeg}

            // Convert RPM to ball exit speed in inches/sec.
            // Surface speed = RPM * pi * diameter / 60
            // Ball speed    = surface speed * efficiency factor
            double ballSpeedIps = params[0]
                    * Math.PI * Constant.SHOOTER_WHEEL_DIAMETER_INCH / 60.0
                    * Constant.BALL_SPEED_EFFICIENCY;

            // Horizontal velocity component of the ball.
            // Hood angle is measured FROM VERTICAL, so horizontal = speed * sin(angle).
            double hoodRad = Math.toRadians(params[1]);
            double horizontalSpeed = ballSpeedIps * Math.sin(hoodRad);

            // Flight time = horizontal distance / horizontal speed.
            // Guard against divide-by-zero if params are invalid.
            double timeOfFlight = (horizontalSpeed > 0.01) ? currentDist / horizontalSpeed : 0.0;

            // Total drift time = delay in mechanism + ball in air.
            double totalDrift = Constant.BALL_TRANSFER_TIME + timeOfFlight;

            // Virtual goal: where does the goal appear to be, accounting for robot motion?
            // The ball inherits the turret's velocity, so from the ball's frame the goal
            // drifts by (turretVelocity * totalDrift) during the shot. We aim at the
            // apparent (virtual) position.
            virtGoalX = goalX - scaledVelX * totalDrift;
            virtGoalY = goalY - scaledVelY * totalDrift;

            // Update distance estimate for next iteration
            currentDist = Math.hypot(virtGoalX - turretX, virtGoalY - turretY);
        }

        // Turret angular velocity needed to track the virtual goal.
        // d/dt[atan2(y,x)] = (y'·x − x'·y) / (x² + y²)
        // Subtract robot omega since the turret rotates with the robot body.
        //
        // CLOSE-RANGE BLOWUP FIX:
        // When dist is small, distSq is tiny, so (vel × dist) / distSq → huge.
        // Example: 5 inches away, 10 in/s strafe → turretOmega = 2 rad/s → 11° lookahead.
        // At 3 inches it's 9× worse. The turret gets commanded 60-90° off.
        // Fix: disable turretOmega entirely below COMP_MIN_DIST (same cutoff as compScale).
        // Also hard-clamp to MAX_TURRET_OMEGA_DEG_S to catch any mid-range residual.

        double dx = virtGoalX - turretX;
        double dy = virtGoalY - turretY;
        double distSq = dx * dx + dy * dy;
        double turretOmega = 0.0;
        double lookaheadMinDistSq = Constant.COMP_MIN_DIST * Constant.COMP_MIN_DIST;
        if (distSq > lookaheadMinDistSq) {
            turretOmega = (turretVelY * dx - turretVelX * dy) / distSq - omega;
            double maxOmega = Math.toRadians(Constant.MAX_TURRET_OMEGA_DEG_S);
            turretOmega = Math.max(-maxOmega, Math.min(maxOmega, turretOmega));
        }

        // LATERAL_COMP_BOOST can push the virtual goal far outside the table (e.g. 200+ inches)
        // The angle (virtGoalX/Y) still uses the full virtual distance — only the RPM input is clamped.
        double tableMin = Constant.SHOOTING_TABLE.firstKey();
        double tableMax = Constant.SHOOTING_TABLE.lastKey();
        double clampedDist = Math.max(tableMin, Math.min(tableMax, currentDist));

        return new ShotSolution(virtGoalX, virtGoalY, clampedDist, turretOmega, compScale);
    }

    // Interpolate RPM and hood angle from the shooting table.
    private static double[] getShotParams(double distInch) {
        Map.Entry<Double, double[]> low  = Constant.SHOOTING_TABLE.floorEntry(distInch);
        Map.Entry<Double, double[]> high = Constant.SHOOTING_TABLE.ceilingEntry(distInch);

        if (low != null && high != null && !low.equals(high)) {
            double factor = (distInch - low.getKey()) / (high.getKey() - low.getKey());
            double rpm   = low.getValue()[0] + (high.getValue()[0] - low.getValue()[0]) * factor;
            double angle = low.getValue()[1] + (high.getValue()[1] - low.getValue()[1]) * factor;
            return new double[]{rpm, angle};
        } else if (low != null) {
            return new double[]{low.getValue()[0], low.getValue()[1]};
        } else if (high != null) {
            return new double[]{high.getValue()[0], high.getValue()[1]};
        }
        return new double[]{Double.NaN, Double.NaN};
    }
}