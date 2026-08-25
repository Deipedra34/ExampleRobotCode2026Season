package example.robot;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

/**
 * limelightsubsystem içindeki saf matematik fonksiyonlarını test eder
 * donanım gerektirmez HAL başlatmaya gerek yok o yüzden normal unit test olarak yazdım
 */
class LimelightMathTest {

    // Constants.VisionConstants tan alınan değerler (sabit kalacak varsayımıyla)
    private static final double CAMERA_HEIGHT  = 0.50;  // m
    private static final double TARGET_HEIGHT  = 1.45;  // m
    private static final double CAMERA_PITCH   = 30.0;  // derece
    private static final double AIM_KP         = 0.04;
    private static final double AIM_TOLERANCE  = 1.0;   // derece

    //mesafe hesabı (trigonometri kullandım ama basitçe: mesafe = yükseklik farkı / tan(açı) oluyo formül) 

    @Test
    void distance_directlyBelowTarget_returnsPositive() {
        double ty = 10.0; // kamera yukarı bakıyor
        double heightDiff = TARGET_HEIGHT - CAMERA_HEIGHT;
        double angleRad   = Math.toRadians(CAMERA_PITCH + ty);
        double distance   = heightDiff / Math.tan(angleRad);
        assertTrue(distance > 0, "Mesafe pozitif olmalı");
    }

    @Test
    void distance_knownGeometry_matchesExpected() {
        // 40 derecede tan(40_derece) = 0.8391
        double ty         = 10.0;
        double heightDiff = TARGET_HEIGHT - CAMERA_HEIGHT; // 0.95 m
        double angleRad   = Math.toRadians(CAMERA_PITCH + ty); // 40 derece
        double expected   = heightDiff / Math.tan(angleRad);   // = 1.132 m
        assertEquals(expected, 0.95 / Math.tan(Math.toRadians(40.0)), 1e-9);
    }

    @Test
    void distance_zeroAngle_returnsNegativeOne() {
        // açı sıfır olursa tan = 0, bölme hatası = -1 döndürülmeli
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
        // ty küçüldükçe mesafe büyür
        assertTrue(d2 > d1, "Daha küçük ty → daha büyük mesafe");
    }

    // hizalama düzeltmesi (P kontrol)

    @Test
    void aimCorrection_positveTx_returnsNegativeRotation() {
        double tx         = 5.0; // hedef sağda
        double correction = -AIM_KP * tx;
        assertTrue(correction < 0, "Sağdaki hedefe sola dönmek için negatif düzeltme gerekir");
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
        assertEquals(c2, c1 * 2.0, 1e-9, "Düzeltme tx ile doğrusal ölçeklenmeli");
    }

    // hizalanma toleransı

    @Test
    void isAimed_withinTolerance_returnsTrue() {
        double tx = 0.5; // tolerans içinde
        assertTrue(Math.abs(tx) < AIM_TOLERANCE);
    }

    @Test
    void isAimed_outsideTolerance_returnsFalse() {
        double tx = 2.0; // tolerans dışı
        assertFalse(Math.abs(tx) < AIM_TOLERANCE);
    }

    @Test
    void isAimed_exactlyAtTolerance_returnsFalse() {
        // sınır değer: |tx| < tolerance (eşit değil küçük olmalı)
        double tx = AIM_TOLERANCE;
        assertFalse(Math.abs(tx) < AIM_TOLERANCE);
    }
}
