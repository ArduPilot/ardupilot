#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

// uncomment this to force the optimisation of this code, note that
// this makes debugging harder
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#pragma GCC optimize("O0")
#else
#pragma GCC optimize("O3")
#endif

#include "SoloGimbalEKF.h"
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;


// Define tuning parameters
const AP_Param::GroupInfo SoloGimbalEKF::var_info[] = {
    AP_GROUPEND
};

// Hash define constants
#define GYRO_BIAS_LIMIT 0.349066f // maximum allowed gyro bias (rad/sec)

// constructor
SoloGimbalEKF::SoloGimbalEKF(const AP_AHRS_NavEKF &ahrs) :
    _ahrs(ahrs),
    states(),
    state(*reinterpret_cast<struct state_elements *>(&states))
{
    AP_Param::setup_object_defaults(this, var_info);
    reset();
}


// complete reset
void SoloGimbalEKF::reset()
{
    memset(&states,0,sizeof(states));
    memset(&gSense,0,sizeof(gSense));
    memset(&Cov,0,sizeof(Cov));
    TiltCorrection = 0;
    StartTime_ms = 0;
    FiltInit = false;
    lastMagUpdate = 0;
    dtIMU = 0;
    innovationIncrement = 0;
    lastInnovation = 0;
}

// run a 9-state EKF used to calculate orientation
void SoloGimbalEKF::RunEKF(float delta_time, const Vector3f &delta_angles, const Vector3f &delta_velocity, const Vector3f &joint_angles)
{
    imuSampleTime_ms = AP_HAL::millis();
    dtIMU = delta_time;

    // initialise variables and constants
    if (!FiltInit) {
        // Note: the start time is initialised to 0 in the constructor
        if (StartTime_ms == 0) {
            StartTime_ms = imuSampleTime_ms;
        }

        // Set data to pre-initialsation defaults
        FiltInit = false;
        newDataMag = false;
        YawAligned = false;
        memset(&state, 0, sizeof(state));
        state.quat[0] = 1.0f;

        bool main_ekf_healthy = false;
        nav_filter_status main_ekf_status;

        if (_ahrs.get_filter_status(main_ekf_status)) {
            if (main_ekf_status.flags.attitude) {
                main_ekf_healthy = true;
            }
        }

        // Wait for gimbal to stabilise to body fixed position for a few seconds before starting small EKF
        // Also wait for navigation EKF to be healthy beasue we are using the velocity output data
        // This prevents jerky gimbal motion from degrading the EKF initial state estimates
        if (imuSampleTime_ms - StartTime_ms < 5000 || !main_ekf_healthy) {
            return;
        }

        Quaternion ned_to_vehicle_quat;
        ned_to_vehicle_quat.from_rotation_matrix(_ahrs.get_rotation_body_to_ned());

        Quaternion vehicle_to_gimbal_quat;
        vehicle_to_gimbal_quat.from_vector312(joint_angles.x,joint_angles.y,joint_angles.z);

        // calculate initial orientation
        state.quat = ned_to_vehicle_quat * vehicle_to_gimbal_quat;

        const float Sigma_velNED = 0.5f; // 1 sigma uncertainty in horizontal velocity components
        const float Sigma_dAngBias = 0.002f*dtIMU; // 1 Sigma uncertainty in delta angle bias (rad)
        const float Sigma_angErr = 0.1f; // 1 Sigma uncertainty in angular misalignment (rad)
        for (uint8_t i=0; i <= 2; i++) Cov[i][i] = sq(Sigma_angErr);
        for (uint8_t i=3; i <= 5; i++) Cov[i][i] = sq(Sigma_velNED);
        for (uint8_t i=6; i <= 8; i++) Cov[i][i] = sq(Sigma_dAngBias);
        FiltInit = true;
        hal.console->printf("\nSoloGimbalEKF Alignment Started\n");

        // Don't run the filter in this timestep because we have already used the delta velocity data to set an initial orientation
        return;
    }

    // We are using IMU data and joint angles from the gimbal
    gSense.gPsi = joint_angles.z; // yaw
    gSense.gPhi = joint_angles.x; // roll
    gSense.gTheta = joint_angles.y; // pitch
    cosPhi = cosf(gSense.gPhi);
    cosTheta = cosf(gSense.gTheta);
    sinPhi = sinf(gSense.gPhi);
    sinTheta = sinf(gSense.gTheta);
    sinPsi = sinf(gSense.gPsi);
    cosPsi = cosf(gSense.gPsi);
    gSense.delAng = delta_angles;
    gSense.delVel = delta_velocity;

    // predict states
    predictStates();

    // predict the covariance
    predictCovariance();

    // fuse SoloGimbalEKF velocity data
    fuseVelocity();

    
    // Align the heading once there has been enough time for the filter to settle and the tilt corrections have dropped below a threshold
    // Force it to align if too much time has lapsed
    if (((((imuSampleTime_ms - StartTime_ms) > 8000 && TiltCorrection < 1e-4f) || (imuSampleTime_ms - StartTime_ms) > 30000)) && !YawAligned) {
        //calculate the initial heading using magnetometer, estimated tilt and declination
        alignHeading();
        YawAligned = true;
        hal.console->printf("\nSoloGimbalEKF Alignment Completed\n");
    }

    // Fuse magnetometer data if  we have new measurements and an aligned heading
    
    readMagData();
    if (newDataMag && YawAligned) {
        fuseCompass();
        newDataMag = false;
    }
    
}

// state prediction
void SoloGimbalEKF::predictStates()
{
    static Vector3f gimDelAngCorrected;
    static Vector3f gimDelAngPrev;

    // NED gravity vector m/s^2
    const Vector3f gravityNED(0, 0, GRAVITY_MSS);

    // apply corrections for bias and coning errors
    // % * - and + operators have been overloaded
    gimDelAngCorrected   = gSense.delAng - state.delAngBias - (gimDelAngPrev % gimDelAngCorrected) * 8.333333e-2f;
    gimDelAngPrev        = gSense.delAng - state.delAngBias;

    // update the quaternions by rotating from the previous attitude through
    // the delta angle rotation quaternion
    state.quat.rotate(gimDelAngCorrected);

    // normalise the quaternions and update the quaternion states
    state.quat.normalize();

    // calculate the sensor to NED cosine matrix
    state.quat.rotation_matrix(Tsn);

    // transform body delta velocities to delta velocities in the nav frame
    // * and + operators have been overloaded
    Vector3f delVelNav  = Tsn*gSense.delVel + gravityNED*dtIMU;

    // sum delta velocities to get velocity
    state.velocity += delVelNav;

    state.delAngBias.x = constrain_float(state.delAngBias.x, -GYRO_BIAS_LIMIT*dtIMU,GYRO_BIAS_LIMIT*dtIMU);
    state.delAngBias.y = constrain_float(state.delAngBias.y, -GYRO_BIAS_LIMIT*dtIMU,GYRO_BIAS_LIMIT*dtIMU);
    state.delAngBias.z = constrain_float(state.delAngBias.z, -GYRO_BIAS_LIMIT*dtIMU,GYRO_BIAS_LIMIT*dtIMU);
}

// covariance prediction using optimised algebraic toolbox expressions
// equivalent to P = F*P*transpose(P) + G*imu_errors*transpose(G) +
// gyro_bias_state_noise
void SoloGimbalEKF::predictCovariance()
{
    float delAngBiasVariance = sq(dtIMU*5E-6f);
    if (YawAligned && !hal.util->get_soft_armed()) {
        delAngBiasVariance *= 4.0f;
    }

    float daxNoise = sq(dtIMU*0.0087f);
    float dayNoise = sq(dtIMU*0.0087f);
    float dazNoise = sq(dtIMU*0.0087f);

    float dvxNoise = sq(dtIMU*0.5f);
    float dvyNoise = sq(dtIMU*0.5f);
    float dvzNoise = sq(dtIMU*0.5f);
    float dvx = gSense.delVel.x;
    float dvy = gSense.delVel.y;
    float dvz = gSense.delVel.z;
    float dax = gSense.delAng.x;
    float day = gSense.delAng.y;
    float daz = gSense.delAng.z;
    float q0 = state.quat[0];
    float q1 = state.quat[1];
    float q2 = state.quat[2];
    float q3 = state.quat[3];
    float dax_b = state.delAngBias.x;
    float day_b = state.delAngBias.y;
    float daz_b = state.delAngBias.z;
    float t1365 = dax*0.5f;
    float t1366 = dax_b*0.5f;
    float t1367 = t1365-t1366;
    float t1368 = day*0.5f;
    float t1369 = day_b*0.5f;
    float t1370 = t1368-t1369;
    float t1371 = daz*0.5f;
    float t1372 = daz_b*0.5f;
    float t1373 = t1371-t1372;
    float t1374 = q2*t1367*0.5f;
    float t1375 = q1*t1370*0.5f;
    float t1376 = q0*t1373*0.5f;
    float t1377 = q2*0.5f;
    float t1378 = q3*t1367*0.5f;
    float t1379 = q1*t1373*0.5f;
    float t1380 = q1*0.5f;
    float t1381 = q0*t1367*0.5f;
    float t1382 = q3*t1370*0.5f;
    float t1383 = q0*0.5f;
    float t1384 = q2*t1370*0.5f;
    float t1385 = q3*t1373*0.5f;
    float t1386 = q0*t1370*0.5f;
    float t1387 = q3*0.5f;
    float t1388 = q1*t1367*0.5f;
    float t1389 = q2*t1373*0.5f;
    float t1390 = t1374+t1375+t1376-t1387;
    float t1391 = t1377+t1378+t1379-t1386;
    float t1392 = q2*t1391*2.0f;
    float t1393 = t1380+t1381+t1382-t1389;
    float t1394 = q1*t1393*2.0f;
    float t1395 = t1383+t1384+t1385-t1388;
    float t1396 = q0*t1395*2.0f;
    float t1403 = q3*t1390*2.0f;
    float t1397 = t1392+t1394+t1396-t1403;
    float t1398 = sq(q0);
    float t1399 = sq(q1);
    float t1400 = sq(q2);
    float t1401 = sq(q3);
    float t1402 = t1398+t1399+t1400+t1401;
    float t1404 = t1374+t1375-t1376+t1387;
    float t1405 = t1377-t1378+t1379+t1386;
    float t1406 = q1*t1405*2.0f;
    float t1407 = -t1380+t1381+t1382+t1389;
    float t1408 = q2*t1407*2.0f;
    float t1409 = t1383-t1384+t1385+t1388;
    float t1410 = q3*t1409*2.0f;
    float t1420 = q0*t1404*2.0f;
    float t1411 = t1406+t1408+t1410-t1420;
    float t1412 = -t1377+t1378+t1379+t1386;
    float t1413 = q0*t1412*2.0f;
    float t1414 = t1374-t1375+t1376+t1387;
    float t1415 = t1383+t1384-t1385+t1388;
    float t1416 = q2*t1415*2.0f;
    float t1417 = t1380-t1381+t1382+t1389;
    float t1418 = q3*t1417*2.0f;
    float t1421 = q1*t1414*2.0f;
    float t1419 = t1413+t1416+t1418-t1421;
    float t1422 = Cov[0][0]*t1397;
    float t1423 = Cov[1][0]*t1411;
    float t1429 = Cov[6][0]*t1402;
    float t1430 = Cov[2][0]*t1419;
    float t1424 = t1422+t1423-t1429-t1430;
    float t1425 = Cov[0][1]*t1397;
    float t1426 = Cov[1][1]*t1411;
    float t1427 = Cov[0][2]*t1397;
    float t1428 = Cov[1][2]*t1411;
    float t1434 = Cov[6][1]*t1402;
    float t1435 = Cov[2][1]*t1419;
    float t1431 = t1425+t1426-t1434-t1435;
    float t1442 = Cov[6][2]*t1402;
    float t1443 = Cov[2][2]*t1419;
    float t1432 = t1427+t1428-t1442-t1443;
    float t1433 = t1398+t1399-t1400-t1401;
    float t1436 = q0*q2*2.0f;
    float t1437 = q1*q3*2.0f;
    float t1438 = t1436+t1437;
    float t1439 = q0*q3*2.0f;
    float t1441 = q1*q2*2.0f;
    float t1440 = t1439-t1441;
    float t1444 = t1398-t1399+t1400-t1401;
    float t1445 = q0*q1*2.0f;
    float t1449 = q2*q3*2.0f;
    float t1446 = t1445-t1449;
    float t1447 = t1439+t1441;
    float t1448 = t1398-t1399-t1400+t1401;
    float t1450 = t1445+t1449;
    float t1451 = t1436-t1437;
    float t1452 = Cov[0][6]*t1397;
    float t1453 = Cov[1][6]*t1411;
    float t1628 = Cov[6][6]*t1402;
    float t1454 = t1452+t1453-t1628-Cov[2][6]*t1419;
    float t1455 = Cov[0][7]*t1397;
    float t1456 = Cov[1][7]*t1411;
    float t1629 = Cov[6][7]*t1402;
    float t1457 = t1455+t1456-t1629-Cov[2][7]*t1419;
    float t1458 = Cov[0][8]*t1397;
    float t1459 = Cov[1][8]*t1411;
    float t1630 = Cov[6][8]*t1402;
    float t1460 = t1458+t1459-t1630-Cov[2][8]*t1419;
    float t1461 = q0*t1390*2.0f;
    float t1462 = q1*t1391*2.0f;
    float t1463 = q3*t1395*2.0f;
    float t1473 = q2*t1393*2.0f;
    float t1464 = t1461+t1462+t1463-t1473;
    float t1465 = q0*t1409*2.0f;
    float t1466 = q2*t1405*2.0f;
    float t1467 = q3*t1404*2.0f;
    float t1474 = q1*t1407*2.0f;
    float t1468 = t1465+t1466+t1467-t1474;
    float t1469 = q1*t1415*2.0f;
    float t1470 = q2*t1414*2.0f;
    float t1471 = q3*t1412*2.0f;
    float t1475 = q0*t1417*2.0f;
    float t1472 = t1469+t1470+t1471-t1475;
    float t1476 = Cov[7][0]*t1402;
    float t1477 = Cov[0][0]*t1464;
    float t1486 = Cov[1][0]*t1468;
    float t1487 = Cov[2][0]*t1472;
    float t1478 = t1476+t1477-t1486-t1487;
    float t1479 = Cov[7][1]*t1402;
    float t1480 = Cov[0][1]*t1464;
    float t1492 = Cov[1][1]*t1468;
    float t1493 = Cov[2][1]*t1472;
    float t1481 = t1479+t1480-t1492-t1493;
    float t1482 = Cov[7][2]*t1402;
    float t1483 = Cov[0][2]*t1464;
    float t1498 = Cov[1][2]*t1468;
    float t1499 = Cov[2][2]*t1472;
    float t1484 = t1482+t1483-t1498-t1499;
    float t1485 = sq(t1402);
    float t1488 = q1*t1390*2.0f;
    float t1489 = q2*t1395*2.0f;
    float t1490 = q3*t1393*2.0f;
    float t1533 = q0*t1391*2.0f;
    float t1491 = t1488+t1489+t1490-t1533;
    float t1494 = q0*t1407*2.0f;
    float t1495 = q1*t1409*2.0f;
    float t1496 = q2*t1404*2.0f;
    float t1534 = q3*t1405*2.0f;
    float t1497 = t1494+t1495+t1496-t1534;
    float t1500 = q0*t1415*2.0f;
    float t1501 = q1*t1417*2.0f;
    float t1502 = q3*t1414*2.0f;
    float t1535 = q2*t1412*2.0f;
    float t1503 = t1500+t1501+t1502-t1535;
    float t1504 = dvy*t1433;
    float t1505 = dvx*t1440;
    float t1506 = t1504+t1505;
    float t1507 = dvx*t1438;
    float t1508 = dvy*t1438;
    float t1509 = dvz*t1440;
    float t1510 = t1508+t1509;
    float t1511 = dvx*t1444;
    float t1551 = dvy*t1447;
    float t1512 = t1511-t1551;
    float t1513 = dvz*t1444;
    float t1514 = dvy*t1446;
    float t1515 = t1513+t1514;
    float t1516 = dvx*t1446;
    float t1517 = dvz*t1447;
    float t1518 = t1516+t1517;
    float t1519 = dvx*t1448;
    float t1520 = dvz*t1451;
    float t1521 = t1519+t1520;
    float t1522 = dvy*t1448;
    float t1552 = dvz*t1450;
    float t1523 = t1522-t1552;
    float t1524 = dvx*t1450;
    float t1525 = dvy*t1451;
    float t1526 = t1524+t1525;
    float t1527 = Cov[7][6]*t1402;
    float t1528 = Cov[0][6]*t1464;
    float t1529 = Cov[7][7]*t1402;
    float t1530 = Cov[0][7]*t1464;
    float t1531 = Cov[7][8]*t1402;
    float t1532 = Cov[0][8]*t1464;
    float t1536 = Cov[8][0]*t1402;
    float t1537 = Cov[1][0]*t1497;
    float t1545 = Cov[0][0]*t1491;
    float t1546 = Cov[2][0]*t1503;
    float t1538 = t1536+t1537-t1545-t1546;
    float t1539 = Cov[8][1]*t1402;
    float t1540 = Cov[1][1]*t1497;
    float t1547 = Cov[0][1]*t1491;
    float t1548 = Cov[2][1]*t1503;
    float t1541 = t1539+t1540-t1547-t1548;
    float t1542 = Cov[8][2]*t1402;
    float t1543 = Cov[1][2]*t1497;
    float t1549 = Cov[0][2]*t1491;
    float t1550 = Cov[2][2]*t1503;
    float t1544 = t1542+t1543-t1549-t1550;
    float t1553 = Cov[8][6]*t1402;
    float t1554 = Cov[1][6]*t1497;
    float t1555 = Cov[8][7]*t1402;
    float t1556 = Cov[1][7]*t1497;
    float t1557 = Cov[8][8]*t1402;
    float t1558 = Cov[1][8]*t1497;
    float t1560 = dvz*t1433;
    float t1559 = t1507-t1560;
    float t1561 = Cov[0][0]*t1510;
    float t1567 = Cov[2][0]*t1506;
    float t1568 = Cov[1][0]*t1559;
    float t1562 = Cov[3][0]+t1561-t1567-t1568;
    float t1563 = Cov[0][1]*t1510;
    float t1569 = Cov[2][1]*t1506;
    float t1570 = Cov[1][1]*t1559;
    float t1564 = Cov[3][1]+t1563-t1569-t1570;
    float t1565 = Cov[0][2]*t1510;
    float t1571 = Cov[2][2]*t1506;
    float t1572 = Cov[1][2]*t1559;
    float t1566 = Cov[3][2]+t1565-t1571-t1572;
    float t1573 = -t1507+t1560;
    float t1574 = Cov[1][0]*t1573;
    float t1575 = Cov[3][0]+t1561-t1567+t1574;
    float t1576 = Cov[1][1]*t1573;
    float t1577 = Cov[3][1]+t1563-t1569+t1576;
    float t1578 = Cov[1][2]*t1573;
    float t1579 = Cov[3][2]+t1565-t1571+t1578;
    float t1580 = Cov[0][6]*t1510;
    float t1581 = Cov[0][7]*t1510;
    float t1582 = Cov[0][8]*t1510;
    float t1583 = Cov[1][0]*t1518;
    float t1584 = Cov[2][0]*t1512;
    float t1592 = Cov[0][0]*t1515;
    float t1585 = Cov[4][0]+t1583+t1584-t1592;
    float t1586 = Cov[1][1]*t1518;
    float t1587 = Cov[2][1]*t1512;
    float t1593 = Cov[0][1]*t1515;
    float t1588 = Cov[4][1]+t1586+t1587-t1593;
    float t1589 = Cov[1][2]*t1518;
    float t1590 = Cov[2][2]*t1512;
    float t1594 = Cov[0][2]*t1515;
    float t1591 = Cov[4][2]+t1589+t1590-t1594;
    float t1595 = dvxNoise*t1433*t1447;
    float t1596 = Cov[1][6]*t1518;
    float t1597 = Cov[2][6]*t1512;
    float t1598 = Cov[4][6]+t1596+t1597-Cov[0][6]*t1515;
    float t1599 = Cov[1][7]*t1518;
    float t1600 = Cov[2][7]*t1512;
    float t1601 = Cov[4][7]+t1599+t1600-Cov[0][7]*t1515;
    float t1602 = Cov[1][8]*t1518;
    float t1603 = Cov[2][8]*t1512;
    float t1604 = Cov[4][8]+t1602+t1603-Cov[0][8]*t1515;
    float t1605 = Cov[2][0]*t1526;
    float t1606 = Cov[0][0]*t1523;
    float t1614 = Cov[1][0]*t1521;
    float t1607 = Cov[5][0]+t1605+t1606-t1614;
    float t1608 = Cov[2][1]*t1526;
    float t1609 = Cov[0][1]*t1523;
    float t1615 = Cov[1][1]*t1521;
    float t1610 = Cov[5][1]+t1608+t1609-t1615;
    float t1611 = Cov[2][2]*t1526;
    float t1612 = Cov[0][2]*t1523;
    float t1616 = Cov[1][2]*t1521;
    float t1613 = Cov[5][2]+t1611+t1612-t1616;
    float t1617 = dvzNoise*t1438*t1448;
    float t1618 = dvyNoise*t1444*t1450;
    float t1619 = Cov[2][6]*t1526;
    float t1620 = Cov[0][6]*t1523;
    float t1621 = Cov[5][6]+t1619+t1620-Cov[1][6]*t1521;
    float t1622 = Cov[2][7]*t1526;
    float t1623 = Cov[0][7]*t1523;
    float t1624 = Cov[5][7]+t1622+t1623-Cov[1][7]*t1521;
    float t1625 = Cov[2][8]*t1526;
    float t1626 = Cov[0][8]*t1523;
    float t1627 = Cov[5][8]+t1625+t1626-Cov[1][8]*t1521;
    float nextCov[9][9];
    nextCov[0][0] = daxNoise*t1485+t1397*t1424+t1411*t1431-t1419*t1432-t1402*t1454;
    nextCov[1][0] = -t1397*t1478-t1411*t1481+t1419*t1484+t1402*(t1527+t1528-Cov[1][6]*t1468-Cov[2][6]*t1472);
    nextCov[2][0] = -t1397*t1538-t1411*t1541+t1419*t1544+t1402*(t1553+t1554-Cov[0][6]*t1491-Cov[2][6]*t1503);
    nextCov[3][0] = -t1402*(Cov[3][6]+t1580-Cov[2][6]*t1506-Cov[1][6]*t1559)+t1397*t1562+t1411*t1564-t1419*t1566;
    nextCov[4][0] = t1397*t1585+t1411*t1588-t1402*t1598-t1419*t1591;
    nextCov[5][0] = t1397*t1607+t1411*t1610-t1402*t1621-t1419*t1613;
    nextCov[6][0] = -t1628+Cov[6][0]*t1397+Cov[6][1]*t1411-Cov[6][2]*t1419;
    nextCov[7][0] = -t1527+Cov[7][0]*t1397+Cov[7][1]*t1411-Cov[7][2]*t1419;
    nextCov[8][0] = -t1553+Cov[8][0]*t1397+Cov[8][1]*t1411-Cov[8][2]*t1419;
    nextCov[0][1] = -t1402*t1457-t1424*t1464+t1431*t1468+t1432*t1472;
    nextCov[1][1] = dayNoise*t1485+t1464*t1478-t1468*t1481-t1472*t1484+t1402*(t1529+t1530-Cov[1][7]*t1468-Cov[2][7]*t1472);
    nextCov[2][1] = t1464*t1538-t1468*t1541-t1472*t1544+t1402*(t1555+t1556-Cov[0][7]*t1491-Cov[2][7]*t1503);
    nextCov[3][1] = -t1402*(Cov[3][7]+t1581-Cov[2][7]*t1506-Cov[1][7]*t1559)-t1464*t1562+t1468*t1564+t1472*t1566;
    nextCov[4][1] = -t1402*t1601-t1464*t1585+t1468*t1588+t1472*t1591;
    nextCov[5][1] = -t1402*t1624-t1464*t1607+t1468*t1610+t1472*t1613;
    nextCov[6][1] = -t1629-Cov[6][0]*t1464+Cov[6][1]*t1468+Cov[6][2]*t1472;
    nextCov[7][1] = -t1529-Cov[7][0]*t1464+Cov[7][1]*t1468+Cov[7][2]*t1472;
    nextCov[8][1] = -t1555-Cov[8][0]*t1464+Cov[8][1]*t1468+Cov[8][2]*t1472;
    nextCov[0][2] = -t1402*t1460-t1431*t1497+t1432*t1503+t1491*(t1422+t1423-t1429-t1430);
    nextCov[1][2] = -t1478*t1491+t1481*t1497-t1484*t1503+t1402*(t1531+t1532-Cov[1][8]*t1468-Cov[2][8]*t1472);
    nextCov[2][2] = dazNoise*t1485-t1491*t1538+t1497*t1541-t1503*t1544+t1402*(t1557+t1558-Cov[0][8]*t1491-Cov[2][8]*t1503);
    nextCov[3][2] = -t1402*(Cov[3][8]+t1582-Cov[2][8]*t1506-Cov[1][8]*t1559)+t1491*t1562-t1497*t1564+t1503*t1566;
    nextCov[4][2] = -t1402*t1604+t1491*t1585-t1497*t1588+t1503*t1591;
    nextCov[5][2] = -t1402*t1627+t1491*t1607-t1497*t1610+t1503*t1613;
    nextCov[6][2] = -t1630+Cov[6][0]*t1491-Cov[6][1]*t1497+Cov[6][2]*t1503;
    nextCov[7][2] = -t1531+Cov[7][0]*t1491-Cov[7][1]*t1497+Cov[7][2]*t1503;
    nextCov[8][2] = -t1557+Cov[8][0]*t1491-Cov[8][1]*t1497+Cov[8][2]*t1503;
    nextCov[0][3] = Cov[0][3]*t1397+Cov[1][3]*t1411-Cov[2][3]*t1419-Cov[6][3]*t1402-t1432*t1506+t1510*(t1422+t1423-t1429-t1430)-t1559*(t1425+t1426-t1434-t1435);
    nextCov[1][3] = -Cov[0][3]*t1464-Cov[7][3]*t1402+Cov[1][3]*t1468+Cov[2][3]*t1472-t1478*t1510+t1484*t1506+t1481*t1559;
    nextCov[2][3] = -Cov[8][3]*t1402+Cov[0][3]*t1491-Cov[1][3]*t1497+Cov[2][3]*t1503-t1510*t1538+t1506*t1544+t1541*t1559;
    nextCov[3][3] = Cov[3][3]+Cov[0][3]*t1510-Cov[2][3]*t1506+Cov[1][3]*t1573-t1506*t1566+t1510*t1575+t1573*t1577+dvxNoise*sq(t1433)+dvyNoise*sq(t1440)+dvzNoise*sq(t1438);
    nextCov[4][3] = Cov[4][3]+t1595-Cov[0][3]*t1515+Cov[1][3]*t1518+Cov[2][3]*t1512+t1510*t1585-t1506*t1591+t1573*t1588-dvyNoise*t1440*t1444-dvzNoise*t1438*t1446;
    nextCov[5][3] = Cov[5][3]+t1617+Cov[0][3]*t1523-Cov[1][3]*t1521+Cov[2][3]*t1526+t1510*t1607-t1506*t1613+t1573*t1610-dvxNoise*t1433*t1451-dvyNoise*t1440*t1450;
    nextCov[6][3] = Cov[6][3]-Cov[6][2]*t1506+Cov[6][0]*t1510+Cov[6][1]*t1573;
    nextCov[7][3] = Cov[7][3]-Cov[7][2]*t1506+Cov[7][0]*t1510+Cov[7][1]*t1573;
    nextCov[8][3] = Cov[8][3]-Cov[8][2]*t1506+Cov[8][0]*t1510+Cov[8][1]*t1573;
    nextCov[0][4] = Cov[0][4]*t1397+Cov[1][4]*t1411-Cov[2][4]*t1419-Cov[6][4]*t1402-t1424*t1515+t1432*t1512+t1518*(t1425+t1426-t1434-t1435);
    nextCov[1][4] = -Cov[0][4]*t1464-Cov[7][4]*t1402+Cov[1][4]*t1468+Cov[2][4]*t1472+t1478*t1515-t1484*t1512-t1481*t1518;
    nextCov[2][4] = -Cov[8][4]*t1402+Cov[0][4]*t1491-Cov[1][4]*t1497+Cov[2][4]*t1503+t1515*t1538-t1512*t1544-t1518*t1541;
    nextCov[3][4] = Cov[3][4]+t1595+Cov[0][4]*t1510-Cov[2][4]*t1506+Cov[1][4]*t1573-t1515*t1575+t1512*t1579+t1518*t1577-dvyNoise*t1440*t1444-dvzNoise*t1438*t1446;
    nextCov[4][4] = Cov[4][4]-Cov[0][4]*t1515+Cov[1][4]*t1518+Cov[2][4]*t1512-t1515*t1585+t1512*t1591+t1518*t1588+dvxNoise*sq(t1447)+dvyNoise*sq(t1444)+dvzNoise*sq(t1446);
    nextCov[5][4] = Cov[5][4]+t1618+Cov[0][4]*t1523-Cov[1][4]*t1521+Cov[2][4]*t1526-t1515*t1607+t1512*t1613+t1518*t1610-dvxNoise*t1447*t1451-dvzNoise*t1446*t1448;
    nextCov[6][4] = Cov[6][4]+Cov[6][2]*t1512-Cov[6][0]*t1515+Cov[6][1]*t1518;
    nextCov[7][4] = Cov[7][4]+Cov[7][2]*t1512-Cov[7][0]*t1515+Cov[7][1]*t1518;
    nextCov[8][4] = Cov[8][4]+Cov[8][2]*t1512-Cov[8][0]*t1515+Cov[8][1]*t1518;
    nextCov[0][5] = Cov[0][5]*t1397+Cov[1][5]*t1411-Cov[2][5]*t1419-Cov[6][5]*t1402+t1424*t1523-t1431*t1521+t1526*(t1427+t1428-t1442-t1443);
    nextCov[1][5] = -Cov[0][5]*t1464-Cov[7][5]*t1402+Cov[1][5]*t1468+Cov[2][5]*t1472-t1478*t1523+t1481*t1521-t1484*t1526;
    nextCov[2][5] = -Cov[8][5]*t1402+Cov[0][5]*t1491-Cov[1][5]*t1497+Cov[2][5]*t1503-t1523*t1538+t1521*t1541-t1526*t1544;
    nextCov[3][5] = Cov[3][5]+t1617+Cov[0][5]*t1510-Cov[2][5]*t1506+Cov[1][5]*t1573-t1521*t1577+t1523*t1575+t1526*t1579-dvxNoise*t1433*t1451-dvyNoise*t1440*t1450;
    nextCov[4][5] = Cov[4][5]+t1618-Cov[0][5]*t1515+Cov[1][5]*t1518+Cov[2][5]*t1512+t1523*t1585-t1521*t1588+t1526*t1591-dvxNoise*t1447*t1451-dvzNoise*t1446*t1448;
    nextCov[5][5] = Cov[5][5]+Cov[0][5]*t1523-Cov[1][5]*t1521+Cov[2][5]*t1526+t1523*t1607-t1521*t1610+t1526*t1613+dvxNoise*sq(t1451)+dvyNoise*sq(t1450)+dvzNoise*sq(t1448);
    nextCov[6][5] = Cov[6][5]-Cov[6][1]*t1521+Cov[6][0]*t1523+Cov[6][2]*t1526;
    nextCov[7][5] = Cov[7][5]-Cov[7][1]*t1521+Cov[7][0]*t1523+Cov[7][2]*t1526;
    nextCov[8][5] = Cov[8][5]-Cov[8][1]*t1521+Cov[8][0]*t1523+Cov[8][2]*t1526;
    nextCov[0][6] = t1454;
    nextCov[1][6] = -t1527-t1528+Cov[1][6]*t1468+Cov[2][6]*t1472;
    nextCov[2][6] = -t1553-t1554+Cov[0][6]*t1491+Cov[2][6]*t1503;
    nextCov[3][6] = Cov[3][6]+t1580-Cov[2][6]*t1506+Cov[1][6]*t1573;
    nextCov[4][6] = t1598;
    nextCov[5][6] = t1621;
    nextCov[6][6] = Cov[6][6];
    nextCov[7][6] = Cov[7][6];
    nextCov[8][6] = Cov[8][6];
    nextCov[0][7] = t1457;
    nextCov[1][7] = -t1529-t1530+Cov[1][7]*t1468+Cov[2][7]*t1472;
    nextCov[2][7] = -t1555-t1556+Cov[0][7]*t1491+Cov[2][7]*t1503;
    nextCov[3][7] = Cov[3][7]+t1581-Cov[2][7]*t1506+Cov[1][7]*t1573;
    nextCov[4][7] = t1601;
    nextCov[5][7] = t1624;
    nextCov[6][7] = Cov[6][7];
    nextCov[7][7] = Cov[7][7];
    nextCov[8][7] = Cov[8][7];
    nextCov[0][8] = t1460;
    nextCov[1][8] = -t1531-t1532+Cov[1][8]*t1468+Cov[2][8]*t1472;
    nextCov[2][8] = -t1557-t1558+Cov[0][8]*t1491+Cov[2][8]*t1503;
    nextCov[3][8] = Cov[3][8]+t1582-Cov[2][8]*t1506+Cov[1][8]*t1573;
    nextCov[4][8] = t1604;
    nextCov[5][8] = t1627;
    nextCov[6][8] = Cov[6][8];
    nextCov[7][8] = Cov[7][8];
    nextCov[8][8] = Cov[8][8];

    // Add the gyro bias state noise
    for (uint8_t i=6;i<=8;i++) {
        nextCov[i][i] = nextCov[i][i] + delAngBiasVariance;
    }

    // copy predicted variances whilst constraining to be non-negative
    for (uint8_t index=0; index<=8; index++) {
        if (nextCov[index][index] < 0.0f) {
            Cov[index][index] = 0.0f;
        } else {
            Cov[index][index] = nextCov[index][index];
        }
    }

    // copy elements to covariance matrix whilst enforcing symmetry
    for (uint8_t rowIndex=1; rowIndex<=8; rowIndex++) {
        for (uint8_t colIndex=0; colIndex<=rowIndex-1; colIndex++) {
            Cov[rowIndex][colIndex] = 0.5f*(nextCov[rowIndex][colIndex] + nextCov[colIndex][rowIndex]);
            Cov[colIndex][rowIndex] = Cov[rowIndex][colIndex];
        }
    }

}

// Fuse the SoloGimbalEKF velocity estimates - this enables alevel reference to be maintained during constant turns
void SoloGimbalEKF::fuseVelocity()
{
    if (!_ahrs.have_inertial_nav()) {
        return;
    }

    float R_OBS = 0.25f;
    float innovation[3];
    float varInnov[3];
    Vector3f angErrVec;
    uint8_t stateIndex;
    float K[9];
    // Fuse measurements sequentially
    for (uint8_t obsIndex=0;obsIndex<=2;obsIndex++) {
        stateIndex = 3 + obsIndex;

        // Calculate the velocity measurement innovation using the SoloGimbalEKF estimate as the observation
        // if heading isn't aligned, use zero velocity (static assumption)
        if (YawAligned) {
            Vector3f measVelNED = Vector3f(0,0,0);
            nav_filter_status main_ekf_status;

            if (_ahrs.get_filter_status(main_ekf_status)) {
                if (main_ekf_status.flags.horiz_vel) {
                    _ahrs.get_velocity_NED(measVelNED);
                }
            }

            innovation[obsIndex] = state.velocity[obsIndex] - measVelNED[obsIndex];
        } else {
            innovation[obsIndex] = state.velocity[obsIndex];
        }

        // Zero the attitude error states - they represent the incremental error so must be zero before corrections are applied
        state.angErr.zero();
        // Calculate the innovation variance
        varInnov[obsIndex] = Cov[stateIndex][stateIndex] + R_OBS;
        // Calculate the Kalman gain and correct states, taking advantage of direct state observation
        for (uint8_t rowIndex=0;rowIndex<=8;rowIndex++) {
            K[rowIndex] = Cov[rowIndex][stateIndex]/varInnov[obsIndex];
            states[rowIndex] -= K[rowIndex] * innovation[obsIndex];
        }

        // Store tilt error estimate for external monitoring
        angErrVec = angErrVec + state.angErr;

        // the first 3 states represent the angular error vector where truth = estimate + error. This is is used to correct the estimated quaternion
        // Bring the quaternion state estimate back to 'truth' by adding the error
        state.quat.rotate(state.angErr);

        // re-normalise the quaternion
        state.quat.normalize();

        // Update the covariance
        for (uint8_t rowIndex=0;rowIndex<=8;rowIndex++) {
            for (uint8_t colIndex=0;colIndex<=8;colIndex++) {
                Cov[rowIndex][colIndex] = Cov[rowIndex][colIndex] - K[rowIndex]*Cov[stateIndex][colIndex];
            }
        }

        // force symmetry and constrain diagonals to be non-negative
        fixCovariance();
    }

    // calculate tilt component of angle correction
    TiltCorrection = sqrtf(sq(angErrVec.x) + sq(angErrVec.y));
}

// check for new magnetometer data and update store measurements if available
void SoloGimbalEKF::readMagData()
{
    if (_ahrs.get_compass() &&
        _ahrs.get_compass()->use_for_yaw() &&
        _ahrs.get_compass()->last_update_usec() != lastMagUpdate) {
        // store time of last measurement update
        lastMagUpdate = _ahrs.get_compass()->last_update_usec();

        // read compass data and scale to improve numerical conditioning
        magData = _ahrs.get_compass()->get_field();

        // let other processes know that new compass data has arrived
        newDataMag = true;

    } else {
        newDataMag = false;
    }
}

// Fuse compass measurements from autopilot
void SoloGimbalEKF::fuseCompass()
{
    float q0 = state.quat[0];
    float q1 = state.quat[1];
    float q2 = state.quat[2];
    float q3 = state.quat[3];

    float magX = magData.x;
    float magY = magData.y;
    float magZ = magData.z;

    const float R_MAG = 3e-2f;

    // Calculate observation Jacobian
    float t5695 = sq(q0);
    float t5696 = sq(q1);
    float t5697 = sq(q2);
    float t5698 = sq(q3);
    float t5699 = t5695+t5696-t5697-t5698;
    float t5702 = q0*q2*2.0f;
    float t5703 = q1*q3*2.0f;
    float t5704 = t5702+t5703;
    float t5705 = q0*q3*2.0f;
    float t5707 = q1*q2*2.0f;
    float t5706 = t5705-t5707;
    float t5708 = cosTheta*sinPsi;
    float t5709 = sinPhi*sinTheta*cosPsi;
    float t5710 = t5708+t5709;
    float t5711 = t5705+t5707;
    float t5712 = sinTheta*sinPsi;
    float t5730 = cosTheta*sinPhi*cosPsi;
    float t5713 = t5712-t5730;
    float t5714 = q0*q1*2.0f;
    float t5720 = q2*q3*2.0f;
    float t5715 = t5714-t5720;
    float t5716 = t5695-t5696+t5697-t5698;
    float t5717 = sinTheta*cosPsi;
    float t5718 = cosTheta*sinPhi*sinPsi;
    float t5719 = t5717+t5718;
    float t5721 = cosTheta*cosPsi;
    float t5735 = sinPhi*sinTheta*sinPsi;
    float t5722 = t5721-t5735;
    float t5724 = sinPhi*t5706;
    float t5725 = cosPhi*sinTheta*t5699;
    float t5726 = cosPhi*cosTheta*t5704;
    float t5727 = t5724+t5725-t5726;
    float t5728 = magZ*t5727;
    float t5729 = t5699*t5710;
    float t5731 = t5704*t5713;
    float t5732 = cosPhi*cosPsi*t5706;
    float t5733 = t5729+t5731-t5732;
    float t5734 = magY*t5733;
    float t5736 = t5699*t5722;
    float t5737 = t5704*t5719;
    float t5738 = cosPhi*sinPsi*t5706;
    float t5739 = t5736+t5737+t5738;
    float t5740 = magX*t5739;
    float t5741 = -t5728+t5734+t5740;
    float t5742 = 1.0f/t5741;
    float t5743 = sinPhi*t5716;
    float t5744 = cosPhi*cosTheta*t5715;
    float t5745 = cosPhi*sinTheta*t5711;
    float t5746 = -t5743+t5744+t5745;
    float t5747 = magZ*t5746;
    float t5748 = t5710*t5711;
    float t5749 = t5713*t5715;
    float t5750 = cosPhi*cosPsi*t5716;
    float t5751 = t5748-t5749+t5750;
    float t5752 = magY*t5751;
    float t5753 = t5715*t5719;
    float t5754 = t5711*t5722;
    float t5755 = cosPhi*sinPsi*t5716;
    float t5756 = t5753-t5754+t5755;
    float t5757 = magX*t5756;
    float t5758 = t5747-t5752+t5757;
    float t5759 = t5742*t5758;
    float t5723 = tanf(t5759);
    float t5760 = sq(t5723);
    float t5761 = t5760+1.0f;
    float t5762 = 1.0f/sq(t5741);
    float H_MAG[3];
    H_MAG[0] = -t5761*(t5742*(magZ*(sinPhi*t5715+cosPhi*cosTheta*t5716)+magY*(t5713*t5716+cosPhi*cosPsi*t5715)+magX*(t5716*t5719-cosPhi*sinPsi*t5715))-t5758*t5762*(magZ*(sinPhi*t5704+cosPhi*cosTheta*t5706)+magY*(t5706*t5713+cosPhi*cosPsi*t5704)+magX*(t5706*t5719-cosPhi*sinPsi*t5704)));
    H_MAG[1] =  t5761*(t5742*(magZ*(cosPhi*cosTheta*t5711-cosPhi*sinTheta*t5715)+magY*(t5711*t5713+t5710*t5715)+magX*(t5711*t5719+t5715*t5722))+t5758*t5762*(magZ*(cosPhi*cosTheta*t5699+cosPhi*sinTheta*t5704)+magY*(t5699*t5713-t5704*t5710)+magX*(t5699*t5719-t5704*t5722)));
    H_MAG[2] =  t5761*(t5742*(-magZ*(sinPhi*t5711+cosPhi*sinTheta*t5716)+magY*(t5710*t5716-cosPhi*cosPsi*t5711)+magX*(t5716*t5722+cosPhi*sinPsi*t5711))-t5758*t5762*(magZ*(sinPhi*t5699-cosPhi*sinTheta*t5706)+magY*(t5706*t5710+cosPhi*t5699*cosPsi)+magX*(t5706*t5722-cosPhi*t5699*sinPsi)));

    // Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
    float PH[3];
    float varInnov = R_MAG;
    for (uint8_t rowIndex=0;rowIndex<=2;rowIndex++) {
        PH[rowIndex] = 0.0f;
        for (uint8_t colIndex=0;colIndex<=2;colIndex++) {
            PH[rowIndex] += Cov[rowIndex][colIndex]*H_MAG[colIndex];
        }
        varInnov += H_MAG[rowIndex]*PH[rowIndex];
    }
    float K_MAG[9];
    float varInnovInv = 1.0f / varInnov;
    for (uint8_t rowIndex=0;rowIndex<=8;rowIndex++) {
        K_MAG[rowIndex] = 0.0f;
        for (uint8_t colIndex=0;colIndex<=2;colIndex++) {
            K_MAG[rowIndex] += Cov[rowIndex][colIndex]*H_MAG[colIndex];
        }
        K_MAG[rowIndex] *= varInnovInv;
    }

    // Calculate the innovation
    float innovation = calcMagHeadingInnov();

    // limit the innovation so that initial corrections are not too large
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // correct the state vector
    state.angErr.zero();
    for (uint8_t i=0;i<=8;i++) {
        states[i] -= K_MAG[i] * innovation;
    }

    // the first 3 states represent the angular error vector where truth = estimate + error. This is is used to correct the estimated quaternion
    // Bring the quaternion state estimate back to 'truth' by adding the error
    state.quat.rotate(state.angErr);

    // re-normalise the quaternion
    state.quat.normalize();

    // correct the covariance using P = P - K*H*P taking advantage of the fact that only the first 3 elements in H are non zero
    float HP[9];
    for (uint8_t colIndex=0;colIndex<=8;colIndex++) {
        HP[colIndex] = 0.0f;
        for (uint8_t rowIndex=0;rowIndex<=2;rowIndex++) {
            HP[colIndex] += H_MAG[rowIndex]*Cov[rowIndex][colIndex];
        }
    }
    for (uint8_t rowIndex=0;rowIndex<=8;rowIndex++) {
        for (uint8_t colIndex=0;colIndex<=8;colIndex++) {
            Cov[rowIndex][colIndex] -= K_MAG[rowIndex] * HP[colIndex];
        }
    }

    // force symmetry and constrain diagonals to be non-negative
    fixCovariance();
}

// Perform an initial heading alignment using the magnetic field and assumed declination
void SoloGimbalEKF::alignHeading()
{
    // calculate the correction rotation vector in NED frame
    Vector3f deltaRotNED = Vector3f(0,0,-calcMagHeadingInnov());

    // rotate into sensor frame
    Vector3f angleCorrection = Tsn.transposed()*deltaRotNED;

    // apply the correction to the quaternion state
    // Bring the quaternion state estimate back to 'truth' by adding the error
    state.quat.rotate(angleCorrection);

    // re-normalize the quaternion
    state.quat.normalize();
}


// Calculate magnetic heading innovation
float SoloGimbalEKF::calcMagHeadingInnov()
{
    // Define rotation from magnetometer to sensor using a 312 rotation sequence
    Matrix3f Tms;
    Tms[0][0] = cosTheta*cosPsi-sinPsi*sinPhi*sinTheta;
    Tms[1][0] = -sinPsi*cosPhi;
    Tms[2][0] = cosPsi*sinTheta+cosTheta*sinPsi*sinPhi;
    Tms[0][1] = cosTheta*sinPsi+cosPsi*sinPhi*sinTheta;
    Tms[1][1] = cosPsi*cosPhi;
    Tms[2][1] = sinPsi*sinTheta-cosTheta*cosPsi*sinPhi;
    Tms[0][2] = -sinTheta*cosPhi;
    Tms[1][2] = sinPhi;
    Tms[2][2] = cosTheta*cosPhi;

    // get earth magnetic field estimate from main ekf if available to take advantage of main ekf magnetic field learning
    Vector3f earth_magfield = Vector3f(0,0,0);
    _ahrs.get_mag_field_NED(earth_magfield);

    float declination;
    if (!earth_magfield.is_zero()) {
        declination = atan2f(earth_magfield.y,earth_magfield.x);
    } else {
        declination = _ahrs.get_compass()->get_declination();
    }

    Vector3f body_magfield = Vector3f(0,0,0);
    _ahrs.get_mag_field_correction(body_magfield);

    // Define rotation from magnetometer to NED axes
    Matrix3f Tmn = Tsn*Tms;

    // rotate magentic field measured at top plate into NED axes afer applying bias values learnt by main EKF
    Vector3f magMeasNED = Tmn*(magData - body_magfield);

    // calculate the innovation where the predicted measurement is the angle wrt magnetic north of the horizontal component of the measured field
    float innovation = atan2f(magMeasNED.y,magMeasNED.x) - declination;

    // wrap the innovation so it sits on the range from +-pi
    if (innovation > M_PI) {
        innovation = innovation - 2*M_PI;
    } else if (innovation < -M_PI) {
        innovation = innovation + 2*M_PI;
    }

    // Unwrap so that a large yaw gyro bias offset that causes the heading to wrap does not lead to continual uncontrolled heading drift
    if (innovation - lastInnovation > M_PI) {
        // Angle has wrapped in the positive direction to subtract an additional 2*Pi
        innovationIncrement -= 2*M_PI;
    } else if (innovation -innovationIncrement < -M_PI) {
        // Angle has wrapped in the negative direction so add an additional 2*Pi
        innovationIncrement += 2*M_PI;
    }
    lastInnovation = innovation;

    return innovation + innovationIncrement;
}

// Force symmmetry and non-negative diagonals on state covarinace matrix
void SoloGimbalEKF::fixCovariance()
{
    // force symmetry
    for (uint8_t rowIndex=1; rowIndex<=8; rowIndex++) {
        for (uint8_t colIndex=0; colIndex<=rowIndex-1; colIndex++) {
            Cov[rowIndex][colIndex] = 0.5f*(Cov[rowIndex][colIndex] + Cov[colIndex][rowIndex]);
            Cov[colIndex][rowIndex] = Cov[rowIndex][colIndex];
        }
    }

    // constrain diagonals to be non-negative
    for (uint8_t index=1; index<=8; index++) {
        if (Cov[index][index] < 0.0f) {
            Cov[index][index] = 0.0f;
        }
    }
}

// return data for debugging EKF
void SoloGimbalEKF::getDebug(float &tilt, Vector3f &velocity, Vector3f &euler, Vector3f &gyroBias) const
{
    tilt = TiltCorrection;
    velocity = state.velocity;
    state.quat.to_euler(euler.x, euler.y, euler.z);
    if (dtIMU < 1.0e-6f) {
        gyroBias.zero();
    } else {
        gyroBias = state.delAngBias / dtIMU;
    }
}

// get gyro bias data
void SoloGimbalEKF::getGyroBias(Vector3f &gyroBias) const
{
    if (dtIMU < 1.0e-6f) {
        gyroBias.zero();
    } else {
        gyroBias = state.delAngBias / dtIMU;
    }
}

void SoloGimbalEKF::setGyroBias(const Vector3f &gyroBias)
{
    if (dtIMU < 1.0e-6f) {
        return;
    }
    state.delAngBias = gyroBias * dtIMU;
}


// get quaternion data
void SoloGimbalEKF::getQuat(Quaternion &quat) const
{
    quat = state.quat;
}

// get filter status - true is aligned
bool SoloGimbalEKF::getStatus() const
{
    float run_time = AP_HAL::millis() - StartTime_ms;
    return  YawAligned && (run_time > 15000);
}

#endif // HAL_CPU_CLASS
