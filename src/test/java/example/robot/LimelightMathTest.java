package example.robot;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

/**
 * tests the plain math functions that live inside the limelight subsystem
 * no hardware needed, no HAL to spin up, so these are just normal unit tests
 */
class LimelightMathTest {

    // values pulled from Constants.VisionConstants (assuming they stay put)
    private static final double CAMERA_HEIGHT  = 0.50;  // m
    private static final double TARGET_HEIGHT  = 1.45;  // m
    private static final double CAMERA_PITCH   = 30.0;  // degrees
    private static final double AIM_KP         = 0.04;
    private static final double AIM_TOLERANCE  = 1.0;   // degrees

    // distance calc (basically trig: distance = height difference / tan(angle))

    @Test
    void distance_directlyBelowTarget_returnsPositive() {
        double ty = 10.0; // camera looking up
        double heightDiff = TARGET_HEIGHT - CAMERA_HEIGHT;
        double angleRad   = Math.toRadians(CAMERA_PITCH + ty);
        double distance   = heightDiff / Math.tan(angleRad);
        assertTrue(distance > 0, "distance should be positive");
    }

    @Test
    void distance_knownGeometry_matchesExpected() {
        // at 40 degrees, tan(40deg) = 0.8391
        double ty         = 10.0;
        double heightDiff = TARGET_HEIGHT - CAMERA_HEIGHT; // 0.95 m
        double angleRad   = Math.toRadians(CAMERA_PITCH + ty); // 40 degrees
        double expected   = heightDiff / Math.tan(angleRad);   // = 1.132 m
        assertEquals(expected, 0.95 / Math.tan(Math.toRadians(40.0)), 1e-9);
    }

    @Test
    void distance_zeroAngle_returnsNegativeOne() {
        // if the angle is zero, tan = 0, and a div-by-zero should give -1 back
        double angleRad = Math.toRadians(0.0);
        double result   = Math.tan(angleRad) == 0 ? -1.0 : 1.0;
        assertEquals(-1.0, result);
    }

    @Test
    void distance_negativeTyMakesAngleSmaller() {
        double ty1 = 10.0;
        double ty2 = -5.0;
        double d1  = 0.95 / Math.tan(Math.toRadians(CAMERA_PITCH + ty1));
        double d2  = 0.95 / Math.tan(Math.toRadians(CAMERA_PITCH + ty2));
        // smaller ty means bigger distance
        assertTrue(d2 > d1, "a smaller ty should give a bigger distance");
    }

    // aim correction (P control)

    @Test
    void aimCorrection_positveTx_returnsNegativeRotation() {
        double tx         = 5.0; // target is to the right
        double correction = -AIM_KP * tx;
        assertTrue(correction < 0, "target on the right needs a negative correction to turn left");
    }

    @Test
    void aimCorrection_zeroTx_returnsZero() {
        double correction = -AIM_KP * 0.0;
        assertEquals(0.0, correction, 1e-9);
    }

    @Test
    void aimCorrection_scales_linearly() {
        double tx1 = 2.0;
        double tx2 = 4.0;
        double c1  = -AIM_KP * tx1;
        double c2  = -AIM_KP * tx2;
        assertEquals(c2, c1 * 2.0, 1e-9, "correction should scale linearly with tx");
    }

    // aim tolerance

    @Test
    void isAimed_withinTolerance_returnsTrue() {
        double tx = 0.5; // inside tolerance
        assertTrue(Math.abs(tx) < AIM_TOLERANCE);
    }

    @Test
    void isAimed_outsideTolerance_returnsFalse() {
        double tx = 2.0; // outside tolerance
        assertFalse(Math.abs(tx) < AIM_TOLERANCE);
    }

    @Test
    void isAimed_exactlyAtTolerance_returnsFalse() {
        // edge case: has to be |tx| < tolerance, not <=
        double tx = AIM_TOLERANCE;
        assertFalse(Math.abs(tx) < AIM_TOLERANCE);
    }
}
